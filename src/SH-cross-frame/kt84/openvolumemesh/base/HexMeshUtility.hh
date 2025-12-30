#pragma once
#include <utility>

namespace kt84 {
    namespace openvolumemesh {
        // assumes Utility is in the derivation tree
        template <class Base>
        struct HexMeshUtility : public Base {
            OpenVolumeMesh::HalfFaceHandle opposite_halfface_handle_in_cell(OpenVolumeMesh::HalfFaceHandle hf) const {
                if (Base::is_boundary(hf))
                    return {};
                return Base::opposite_halfface_handle_in_cell(hf, Base::incident_cell(hf));
            }
            // |   input: v, hf
            // |   output: w, he
            // |
            // |      (w)--------*
            // |      /         /|
            // |   (he)        / |
            // |    /         /  |
            // |  (v)--------*   |
            // |   |         |   *
            // |   |         |  / 
            // |   |  (hf)   | /  
            // |   |         |/
            // |   *---------*
            std::pair<OpenVolumeMesh::VertexHandle, OpenVolumeMesh::HalfEdgeHandle> opposite_vertex_handle_in_cell(OpenVolumeMesh::VertexHandle v, OpenVolumeMesh::HalfFaceHandle hf) const {
                if (Base::is_boundary(hf))
                    return {};
                auto hf2 = opposite_halfface_handle_in_cell(hf);
                for (auto w : Base::halfface_vertices(hf2)) {
                    auto he = Base::halfedge(v, w);
                    if (he.is_valid())
                        return { w, he };
                }
                return {};  // shouldn't reach here
            }
        };
    }
}
