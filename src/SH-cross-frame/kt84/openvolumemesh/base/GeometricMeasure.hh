#pragma once
#include <OpenVolumeMesh/Core/OpenVolumeMeshHandle.hh>

namespace kt84 {
    namespace openvolumemesh {
        // assumes Utility is in the derivation tree
        template <class Base>
        struct GeometricMeasure : public Base {
            using PointT = typename Base::PointT;
            using Scalar = typename PointT::value_type;
            Scalar triangle_area(OpenVolumeMesh::FaceHandle f) const {
                if (Base::valence(f) != 3)
                    return -1;
                // Heron's formula (http://en.wikipedia.org/wiki/Heron's_formula)
                PointT p[3];
                int i = 0;
                for (auto v : Base::face_vertices(f))
                    p[i++] = Base::vertex(v);
                Scalar a = (p[1] - p[0]).norm();
                Scalar b = (p[2] - p[1]).norm();
                Scalar c = (p[0] - p[2]).norm();
                Scalar s = (a + b + c) / 2;
                return std::sqrt(s * (s - a) * (s - b) * (s - c));
            }
            Scalar triangle_area(OpenVolumeMesh::HalfFaceHandle hf) const {
                return triangle_area(Base::face_handle(hf));
            }
            Scalar tetrahedron_volume(OpenVolumeMesh::CellHandle cell) const {
                if (Base::valence(cell) != 4)
                    return -1;
                // Heron-like formula (http://en.wikipedia.org/wiki/Tetrahedron#Heron-type_formula_for_the_volume_of_a_tetrahedron)
                Scalar UVW[3], uvw[3];
                int i = 0;
                auto f = Base::cell_faces(cell).front();
                for (auto e1 : Base::face_edges(f)) {
                    for (auto e2 : Base::cell_edges(cell)) {
                        if (!Base::is_adjacent(e1, e2)) {
                            UVW[i] = Base::length(e1);
                            uvw[i++] = Base::length(e2);
                            break;
                        }
                    }
                }
                Scalar U = UVW[0];
                Scalar V = UVW[1];
                Scalar W = UVW[2];
                Scalar u = uvw[0];
                Scalar v = uvw[1];
                Scalar w = uvw[2];
                Scalar X = (w - U + v) * (U + v + w);
                Scalar Y = (u - V + w) * (V + w + u);
                Scalar Z = (v - W + u) * (W + u + v);
                Scalar x = (U - v + w) * (v - w + U);
                Scalar y = (V - w + u) * (w - u + V);
                Scalar z = (W - u + v) * (u - v + W);
                Scalar a = std::sqrt(x * Y * Z);
                Scalar b = std::sqrt(y * Z * X);
                Scalar c = std::sqrt(z * X * Y);
                Scalar d = std::sqrt(x * y * z);
                return std::sqrt((-a + b + c + d) * (a - b + c + d) * (a + b - c + d) * (a + b + c - d)) / (192 * u * v * w);
            }
        };
    }
}
