#pragma once
#include <vector>
#include <OpenVolumeMesh/Geometry/VectorT.hh>

namespace kt84 {
    namespace openvolumemesh {
        struct TetFaceNormal_HalfFaceTraits {
            OpenVolumeMesh::Vec3d tetFaceNormal;
        };
        template <class Base>
        struct TetFaceNormal : public Base {
            void tetFaceNormal_compute() {
                for (auto f : Base::faces()) {
                    auto hf = Base::halfface_handle(f, 0);
                    std::vector<OpenVolumeMesh::Vec3d> p;
                    for (auto v : Base::halfface_vertices(hf))
                        p.push_back(Base::vertex(v));
                    OpenVolumeMesh::Vec3d n = ((p[1] - p[0]) % (p[2] - p[0])).normalize();
                    Base::data(hf).tetFaceNormal = n;
                    Base::data(Base::opposite_halfface_handle(hf)).tetFaceNormal = -n;
                }
            }
        };
    }
}
