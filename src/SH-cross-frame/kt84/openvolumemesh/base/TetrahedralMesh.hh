#pragma once
#include <OpenVolumeMesh/Core/OpenVolumeMeshHandle.hh>

namespace kt84 {
    namespace openvolumemesh {
        template <class Base>
        struct TetrahedralMesh : public Base {
            using Base::add_cell;
            OpenVolumeMesh::CellHandle add_cell(const OpenVolumeMesh::VertexHandle& v0, const OpenVolumeMesh::VertexHandle& v1, const OpenVolumeMesh::VertexHandle& v2, const OpenVolumeMesh::VertexHandle& v3, bool topologyCheck = false) {
                /*
                      3-----2
                     / \  ./|
                    /   \/  |
                   /  ./ \  |
                  / ./    \ |
                 / /       \|
                0-----------1
                */
                // copied from <OpenVolumeMesh/Mesh/HexahedralMeshTopologyKernel.cc>
                using namespace OpenVolumeMesh;
            
                // debug mode checks
                assert(Base::has_full_bottom_up_incidences());

                // release mode checks
                if(!Base::has_full_bottom_up_incidences()) {
                    return CellHandle(-1);
                }

                HalfFaceHandle hf0, hf1, hf2, hf3;

                std::vector<VertexHandle> vs;

                // Half-face (0, 1, 2)
                vs.push_back(v0);
                vs.push_back(v1);
                vs.push_back(v2);
                hf0 = Base::halfface(vs); vs.clear();

                // Half-face (3, 1, 0)
                vs.push_back(v3);
                vs.push_back(v1);
                vs.push_back(v0);
                hf1 = Base::halfface(vs); vs.clear();

                // Half-face (3, 2, 1)
                vs.push_back(v3);
                vs.push_back(v2);
                vs.push_back(v1);
                hf2 = Base::halfface(vs); vs.clear();

                // Half-face (3, 0, 2)
                vs.push_back(v3);
                vs.push_back(v0);
                vs.push_back(v2);
                hf3 = Base::halfface(vs); vs.clear();

                if(!hf0.is_valid()) {

                    vs.clear();
                    vs.push_back(v0); vs.push_back(v1); vs.push_back(v2);
                    FaceHandle fh = Base::add_face(vs);
                    hf0 = Base::halfface_handle(fh, 0);
                }

                if(!hf1.is_valid()) {

                    vs.clear();
                    vs.push_back(v3); vs.push_back(v1); vs.push_back(v0);
                    FaceHandle fh = Base::add_face(vs);
                    hf1 = Base::halfface_handle(fh, 0);
                }

                if(!hf2.is_valid()) {

                    vs.clear();
                    vs.push_back(v3); vs.push_back(v2); vs.push_back(v1);
                    FaceHandle fh = Base::add_face(vs);
                    hf2 = Base::halfface_handle(fh, 0);
                }

                if(!hf3.is_valid()) {

                    vs.clear();
                    vs.push_back(v3); vs.push_back(v0); vs.push_back(v2);
                    FaceHandle fh = Base::add_face(vs);
                    hf3 = Base::halfface_handle(fh, 0);
                }

                assert(hf0.is_valid()); assert(hf1.is_valid()); assert(hf2.is_valid()); assert(hf3.is_valid());

                std::vector<HalfFaceHandle> hfs;
                hfs.push_back(hf0); hfs.push_back(hf1); hfs.push_back(hf2); hfs.push_back(hf3);

                if (topologyCheck) {
                    /*
                    * Test if all halffaces are connected and form a two-manifold
                    * => Cell is closed
                    *
                    * This test is simple: The number of involved half-edges has to be
                    * exactly twice the number of involved edges.
                    */

                    std::set<HalfEdgeHandle> incidentHalfedges;
                    std::set<EdgeHandle>     incidentEdges;

                    for(std::vector<HalfFaceHandle>::const_iterator it = hfs.begin(),
                            end = hfs.end(); it != end; ++it) {

                        OpenVolumeMeshFace hface = Base::halfface(*it);
                        for(std::vector<HalfEdgeHandle>::const_iterator he_it = hface.halfedges().begin(),
                                he_end = hface.halfedges().end(); he_it != he_end; ++he_it) {
                            incidentHalfedges.insert(*he_it);
                            incidentEdges.insert(Base::edge_handle(*he_it));
                        }
                    }

                    if(incidentHalfedges.size() != (incidentEdges.size() * 2u)) {
            #ifndef NDEBUG
                        std::cerr << "The specified halffaces are not connected!" << std::endl;
            #endif
                        return Base::InvalidCellHandle;
                    }
                    // The halffaces are now guaranteed to form a two-manifold

                    if(Base::has_face_bottom_up_incidences()) {

                        for(std::vector<HalfFaceHandle>::const_iterator it = hfs.begin(),
                                end = hfs.end(); it != end; ++it) {
                            if(Base::incident_cell(*it) != Base::InvalidCellHandle) {
            #ifndef NDEBUG
                                std::cerr << "Warning: One of the specified half-faces is already incident to another cell!" << std::endl;
            #endif
                                return Base::InvalidCellHandle;
                            }
                        }

                    }

                }

                return Base::add_cell(hfs, false);
            }
            // utilities
            OpenVolumeMesh::EdgeHandle opposite_edge_handle_in_cell(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::EdgeHandle e) {
                if (!Base::is_incident(c, e)) return {};
                for (auto e2 : Base::cell_edges(c))
                    if (!Base::is_adjacent(e, e2)) return e2;
                return {};
            }
            OpenVolumeMesh::HalfFaceHandle opposite_halfface_handle_in_cell(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::VertexHandle v) {
                if (!Base::is_incident(c, v)) return {};
                for (auto hf : Base::cell_halffaces(c))
                    if (!Base::is_adjacent(hf, v)) return hf;
                return {};
            }
            OpenVolumeMesh::FaceHandle opposite_face_handle_in_cell(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::VertexHandle v) {
                return Base::face_handle(opposite_halfface_handle_in_cell(c, v));
            }
        };
    }
}
