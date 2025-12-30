#pragma once
#include <vector>
#include "StaticProperty.hh"

namespace kt84 {
    namespace openvolumemesh {
        template <class Traits, class Base>
        struct DebugInfoTraits {
            struct VertexData;
            struct EdgeData;
            struct FaceData;
            struct CellData;
            struct HalfEdgeData;
            struct HalfFaceData;
            struct VertexData : public Traits::VertexData {
                int dbg_id = -1;
                bool dbg_is_boundary = false;
                const typename Base::PointT* dbg_point = nullptr;
                std::vector<VertexData*> dbg_vertices;
                std::vector<FaceData*> dbg_faces;
                std::vector<CellData*> dbg_cells;
                std::vector<EdgeData*> dbg_edges;
                std::vector<HalfEdgeData*> dbg_outgoing_halfedges;
                std::vector<HalfEdgeData*> dbg_incoming_halfedges;
            };
            struct EdgeData : public Traits::EdgeData {
                int dbg_id = -1;
                bool dbg_is_boundary = false;
                VertexData* dbg_vertex[2];
                HalfEdgeData* dbg_halfedge[2];
                std::vector<CellData*> dbg_cells;
                std::vector<FaceData*> dbg_faces;
            };
            struct FaceData : public Traits::FaceData {
                int dbg_id = -1;
                bool dbg_is_boundary = false;
                HalfFaceData* dbg_halfface[2];
                CellData* dbg_cell[2];
                std::vector<VertexData*> dbg_vertices;
                std::vector<EdgeData*> dbg_edges;
            };
            struct CellData : public Traits::CellData {
                int dbg_id = -1;
                bool dbg_is_boundary = false;
                std::vector<VertexData*> dbg_vertices;
                std::vector<EdgeData*> dbg_edges;
                std::vector<FaceData*> dbg_faces;
                std::vector<HalfFaceData*> dbg_halffaces;
            };
            struct HalfEdgeData : public Traits::HalfEdgeData {
                int dbg_id = -1;
                bool dbg_is_boundary = false;
                VertexData* dbg_from = nullptr;
                VertexData* dbg_to = nullptr;
                EdgeData* dbg_edge = nullptr;
                HalfEdgeData* dbg_opposite = nullptr;
                std::vector<CellData*> dbg_cells;
                std::vector<FaceData*> dbg_faces;
                std::vector<HalfFaceData*> dbg_halffaces;
            };
            struct HalfFaceData : public Traits::HalfFaceData {
                int dbg_id = -1;
                bool dbg_is_boundary = false;
                FaceData* dbg_face = nullptr;
                CellData* dbg_cell = nullptr;
                HalfFaceData* dbg_opposite = nullptr;
                std::vector<VertexData*> dbg_vertices;
                std::vector<EdgeData*> dbg_edges;
                std::vector<HalfEdgeData*> dbg_halfedges;
            };
        };
        // assumes Utility is in the derivation tree
        template <class Traits, class Base>
        struct DebugInfo : public StaticProperty<DebugInfoTraits<Traits, Base>, Base> {
            void debugInfo_get() {
                using ThisBase = StaticProperty<DebugInfoTraits<Traits, Base>, Base>;
                // vertex
                for (auto v : ThisBase::vertices()) {
                    auto& data = ThisBase::data(v);
                    data.dbg_id = v.idx();
                    data.dbg_is_boundary = ThisBase::is_boundary(v);
                    data.dbg_point = &ThisBase::vertex(v);
                    data.dbg_vertices.clear();
                    data.dbg_edges.clear();
                    data.dbg_faces.clear();
                    data.dbg_cells.clear();
                    data.dbg_outgoing_halfedges.clear();
                    data.dbg_incoming_halfedges.clear();
                    for (auto c : ThisBase::vertex_cells(v))
                        data.dbg_cells.push_back(&ThisBase::data(c));
                    for (auto f : ThisBase::vertex_faces(v))
                        data.dbg_faces.push_back(&ThisBase::data(f));
                    for (auto he : ThisBase::outgoing_halfedges(v)) {
                        data.dbg_vertices.push_back(&ThisBase::data(ThisBase::halfedge(he).to_vertex()));
                        auto e = ThisBase::edge_handle(he);
                        data.dbg_edges.push_back(&ThisBase::data(e));
                        data.dbg_outgoing_halfedges.push_back(&ThisBase::data(he));
                        auto he2 = ThisBase::opposite_halfedge_handle(he);
                        data.dbg_incoming_halfedges.push_back(&ThisBase::data(he2));
                    }
                }
                // edge
                for (auto e : ThisBase::edges()) {
                    auto& data = ThisBase::data(e);
                    data.dbg_id = e.idx();
                    data.dbg_is_boundary = ThisBase::is_boundary(e);
                    auto& edge = ThisBase::edge(e);
                    data.dbg_vertex[0] = &ThisBase::data(edge.from_vertex());
                    data.dbg_vertex[1] = &ThisBase::data(edge.to_vertex());
                    data.dbg_halfedge[0] = &ThisBase::data(ThisBase::halfedge_handle(e, 0));
                    data.dbg_halfedge[1] = &ThisBase::data(ThisBase::halfedge_handle(e, 1));
                    data.dbg_cells.clear();
                    data.dbg_faces.clear();
                    for (auto c : ThisBase::edge_cells(e))
                        data.dbg_cells.push_back(&ThisBase::data(c));
                    for (auto f : ThisBase::edge_faces(e))
                        data.dbg_faces.push_back(&ThisBase::data(f));
                }
                // face
                for (auto f : ThisBase::faces()) {
                    auto& data = ThisBase::data(f);
                    data.dbg_id = f.idx();
                    data.dbg_is_boundary = ThisBase::is_boundary(f);
                    auto hf0 = ThisBase::halfface_handle(f, 0);
                    auto hf1 = ThisBase::halfface_handle(f, 1);
                    auto c0 = ThisBase::incident_cell(hf0);
                    auto c1 = ThisBase::incident_cell(hf1);
                    data.dbg_halfface[0] = &ThisBase::data(hf0);
                    data.dbg_halfface[1] = &ThisBase::data(hf1);
                    data.dbg_cell[0] = c0.is_valid() ? &ThisBase::data(c0) : nullptr;
                    data.dbg_cell[1] = c1.is_valid() ? &ThisBase::data(c1) : nullptr;
                    data.dbg_vertices.clear();
                    data.dbg_edges.clear();
                    for (auto v : ThisBase::face_vertices(f))
                        data.dbg_vertices.push_back(&ThisBase::data(v));
                    for (auto e : ThisBase::face_edges(f))
                        data.dbg_edges.push_back(&ThisBase::data(e));
                }
                // cell
                for (auto c : ThisBase::cells()) {
                    auto& data = ThisBase::data(c);
                    data.dbg_id = c.idx();
                    data.dbg_is_boundary = false;
                    for (auto f : ThisBase::cell_faces(c)) {
                        if (ThisBase::is_boundary(f)) {
                            data.dbg_is_boundary = true;
                            break;
                        }
                    }
                    data.dbg_vertices.clear();
                    data.dbg_edges.clear();
                    data.dbg_faces.clear();
                    data.dbg_halffaces.clear();
                    for (auto v : ThisBase::cell_vertices(c))
                        data.dbg_vertices.push_back(&ThisBase::data(v));
                    for (auto e : ThisBase::cell_edges(c))
                        data.dbg_edges.push_back(&ThisBase::data(e));
                    for (auto hf : ThisBase::cell(c).halffaces()) {
                        data.dbg_faces.push_back(&ThisBase::data(Base::face_handle(hf)));
                        data.dbg_halffaces.push_back(&ThisBase::data(hf));
                    }
                }
                // halfedge
                for (auto he : ThisBase::halfedges()) {
                    auto& data = ThisBase::data(he);
                    data.dbg_id = he.idx();
                    data.dbg_is_boundary = ThisBase::is_boundary(he);
                    auto edge = ThisBase::halfedge(he);
                    data.dbg_from = &ThisBase::data(edge.from_vertex());
                    data.dbg_to = &ThisBase::data(edge.to_vertex());
                    data.dbg_edge = &ThisBase::data(ThisBase::edge_handle(he));
                    data.dbg_opposite = &ThisBase::data(ThisBase::opposite_halfedge_handle(he));
                    data.dbg_faces.clear();
                    data.dbg_cells.clear();
                    data.dbg_halffaces.clear();
                    for (auto hf : ThisBase::halfedge_halffaces(he)) {
                        data.dbg_faces.push_back(&ThisBase::data(ThisBase::face_handle(hf)));
                        data.dbg_halffaces.push_back(&ThisBase::data(hf));
                    }
                    for (auto c : ThisBase::halfedge_cells(he))
                        data.dbg_cells.push_back(&ThisBase::data(c));
                }
                // halfface
                for (auto hf : ThisBase::halffaces()) {
                    auto& data = ThisBase::data(hf);
                    data.dbg_id = hf.idx();
                    data.dbg_is_boundary = ThisBase::is_boundary(hf);
                    data.dbg_face = &ThisBase::data(ThisBase::face_handle(hf));
                    auto c = ThisBase::incident_cell(hf);
                    data.dbg_cell = c.is_valid() ? &ThisBase::data(c) : nullptr;
                    data.dbg_opposite = &ThisBase::data(ThisBase::opposite_halfface_handle(hf));
                    data.dbg_vertices.clear();
                    data.dbg_edges.clear();
                    data.dbg_halfedges.clear();
                    for (auto v : ThisBase::halfface_vertices(hf))
                        data.dbg_vertices.push_back(&ThisBase::data(v));
                    auto face = ThisBase::halfface(hf);
                    for (auto he : face.halfedges()) {
                        data.dbg_edges.push_back(&ThisBase::data(ThisBase::edge_handle(he)));
                        data.dbg_halfedges.push_back(&ThisBase::data(he));
                    }
                }
            }
        };
    }
}
