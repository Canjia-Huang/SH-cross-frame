#pragma once
#include <vector>
#include <stack>
#include <memory>
#include <iostream>
#include <algorithm>
#include <Eigen/Geometry>
#include <OpenMesh/Core/Mesh/Handles.hh>
#include "../../eigen_util.hh"
#include "../../vector_cast.hh"

// modifications to Wenzel Jakob's code
namespace kt84 {
    namespace openmesh {
        // interface structs
        struct BVH_Ray {
            Eigen::Vector3d o, d;      // origin, direction
            double mint, maxt;
            BVH_Ray(){}
            BVH_Ray(const Eigen::Vector3d &o, const Eigen::Vector3d &d, double mint = 0, double maxt = std::numeric_limits<double>::infinity()) :
                o(o), d(d), mint(mint), maxt(maxt)
            {}
            Eigen::Vector3d operator()(double t) const { return o + t * d; }
        };
        struct BVH_IntersectInfo {
            OpenMesh::FaceHandle triangle;
            Eigen::Vector3d triangle_bc;        // barycentric coordinates
            double ray_t;
            bool operator<(const BVH_IntersectInfo& rhs) const { return ray_t < rhs.ray_t; }
        };
        // internals
        namespace internal {
            inline bool intersect(const Eigen::AlignedBox3d& aabb, const BVH_Ray& ray) {
                double nearT = -std::numeric_limits<double>::infinity();
                double farT = std::numeric_limits<double>::infinity();
                for (int i = 0; i < 3; i++) {
                    double origin = ray.o[i];
                    double minVal = aabb.min()[i], maxVal = aabb.max()[i];
                    if (ray.d[i] == 0) {
                        if (origin < minVal || origin > maxVal)
                            return false;
                    } else {
                        double t1 = (minVal - origin) / ray.d[i];
                        double t2 = (maxVal - origin) / ray.d[i];
                        if (t1 > t2)
                            std::swap(t1, t2);
                        nearT = std::max(t1, nearT);
                        farT = std::min(t2, farT);
                        if (!(nearT <= farT))
                            return false;
                    }
                }
                return ray.mint <= farT && nearT <= ray.maxt;
            }
            struct BVH_Node {
                std::shared_ptr<BVH_Node> children[2];
                Eigen::AlignedBox3d aabb;
                OpenMesh::FaceHandle triangle;
            };
        }
        template <class Base>
        struct BVH : public Base {
            void bvh_build() {
                std::vector<OpenMesh::FaceHandle> faces;
                faces.reserve(Base::n_faces());
                for (auto f : Base::faces())
                    faces.push_back(f);
                bvh_num_nodes = 0;
                std::cout << "Building BVH for " << faces.size() << " triangles.." << std::endl;
                bvh_root = internal_bvh_build(faces.begin(), faces.end(), 0);
                std::cout << "Done. (" << bvh_num_nodes << " nodes)" << std::endl;
            }
            std::vector<BVH_IntersectInfo> bvh_intersect(const BVH_Ray& ray) const {
                auto node = bvh_root.get();
                std::stack<decltype(node)> stack;
                int nVisited = 0;
                std::vector<BVH_IntersectInfo> result;
                while (true) {
                    nVisited++;
                    if (!internal::intersect(node->aabb, ray)) {
                        if (stack.empty())
                            break;
                        node = util::top_and_pop(stack);
                        continue;
                    }
                    if (!node->triangle.is_valid()) {
                        if (node->children[0] && node->children[1]) {
                            stack.push(node->children[1].get());
                            node = node->children[0].get();
                        } else {
                            node = (node->children[0] ? node->children[0] : node->children[1]).get();
                        }
                    } else {
                        Eigen::Vector3d triangle_bc;
                        double ray_t;
                        if (internal_bvh_intersect(ray, node->triangle, triangle_bc, ray_t)) {
                            result.push_back({ node->triangle, triangle_bc, ray_t });
                        }
                        if (stack.empty())
                            break;
                        node = util::top_and_pop(stack);
                        continue;
                    }
                }
                return result;
            }
            std::vector<OpenMesh::FaceHandle> bvh_find_nearby(const Eigen::Vector3d& pos, double radius) const {
                using namespace Eigen;
                auto node = bvh_root.get();
                std::stack<decltype(node)> stack;
                int nVisited = 0;
                double radius2 = radius*radius;
                std::vector<OpenMesh::FaceHandle> result;
                while (true) {
                    nVisited++;
                    if (node->aabb.squaredExteriorDistance(pos) > radius2) {
                        if (stack.empty())
                            break;
                        node = util::top_and_pop(stack);
                        continue;
                    }
                    if (!node->triangle.is_valid()) {
                        if (node->children[0] && node->children[1]) {
                            stack.push(node->children[1].get());
                            node = node->children[0].get();
                        } else {
                            node = (node->children[0] ? node->children[0] : node->children[1]).get();
                        }
                    } else {
                        Vector3d centroid = Vector3d::Zero();
                        for (auto v : Base::fv_range(node->triangle))
                            centroid += vector_cast<3, Vector3d>(Base::point(v)) / 3.0;
                        Vector3d rel = centroid - pos;
                        if (rel.dot(rel) < radius2)
                            result.push_back(node->triangle);
                        if (stack.empty())
                            break;
                        node = util::top_and_pop(stack);
                        continue;
                    }
                }
                return result;
            }
        private:
            using Iter = std::vector<OpenMesh::FaceHandle>::iterator;
            std::shared_ptr<internal::BVH_Node> internal_bvh_build(Iter start, Iter end, int axis) {
                if (start == end)       // is it supposed to reach here? ask Wenzel
                    return {};
                ++bvh_num_nodes;
                auto node = std::make_shared<internal::BVH_Node>();
                if (start + 1 == end) {
                    node->triangle = *start;
                    for (auto v : Base::fv_range(node->triangle))
                        node->aabb.extend(vector_cast<3, Eigen::Vector3d>(Base::point(v)));
                    return node;
                }
                Iter middle = start + std::distance(start, end) / 2;
                using Point = typename Base::Point;     // workaround for bug in VS2013
                std::nth_element(start, middle, end, [&](OpenMesh::FaceHandle fi, OpenMesh::FaceHandle fj) -> bool {
                    Point accum(0, 0, 0);
                    for (auto v : Base::fv_range(fi))
                        accum += Base::point(v);
                    for (auto v : Base::fv_range(fj))
                        accum -= Base::point(v);
                    return accum[axis] < 0;
                });
                if (node->children[0] = internal_bvh_build(start, middle, (axis + 1) % 3))
                    node->aabb.extend(node->children[0]->aabb);
                if (node->children[1] = internal_bvh_build(middle, end, (axis + 1) % 3))
                    node->aabb.extend(node->children[1]->aabb);
                return node;
            }
            bool internal_bvh_intersect(const BVH_Ray &ray, OpenMesh::FaceHandle triangle, Eigen::Vector3d& triangle_bc, double &ray_t) const {
                using namespace Eigen;
                Vector3d p[3];
                int i = 0;
                for (auto v : Base::fv_range(triangle))
                    p[i++] = vector_cast<3, Vector3d>(Base::point(v));
                Vector3d edge1 = p[1] - p[0], edge2 = p[2] - p[0];
                Vector3d pvec = ray.d.cross(edge2);
                double det = edge1.dot(pvec);
                if (det == 0)
                    return false;
                double inv_det = 1.0 / det;
                Vector3d tvec = ray.o - p[0];
                double u = tvec.dot(pvec) * inv_det;
                if (u < 0.0 || u > 1.0)
                    return false;
                Vector3d qvec = tvec.cross(edge1);
                double v = ray.d.dot(qvec) * inv_det;
                if (v < 0.0 || u + v > 1.0)
                    return false;
                double tempT = edge2.dot(qvec) * inv_det;
                if (tempT < ray.mint || tempT > ray.maxt)
                    return false;
                triangle_bc = Vector3d(1 - u - v, u, v);
                ray_t = tempT;
                return true;
            }
            int bvh_num_nodes;
            std::shared_ptr<internal::BVH_Node> bvh_root;
        };
    }
}
