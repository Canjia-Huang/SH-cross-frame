#pragma once
#include <vector>
#include <OpenVolumeMesh/Core/OpenVolumeMeshHandle.hh>
#include "../range.hh"
#include "../../container_util.hh"

namespace kt84 {
    namespace openvolumemesh {
        template <class Base>
        struct Utility : public Base {
            // incident entities --------------------
            // vertex incidence
            std::vector<OpenVolumeMesh::EdgeHandle> vertex_edges(OpenVolumeMesh::VertexHandle v) const {
                std::vector<OpenVolumeMesh::EdgeHandle> result;
                for (auto he : Base::outgoing_halfedges(v))
                    result.push_back(Base::edge_handle(he));
                return result;
            }
            std::vector<OpenVolumeMesh::FaceHandle> vertex_faces(OpenVolumeMesh::VertexHandle v) const {
                std::vector<OpenVolumeMesh::FaceHandle> result;
                for (auto he : Base::outgoing_halfedges(v))
                    for (auto hf : Base::halfedge_halffaces(he))
                        result.push_back(Base::face_handle(hf));
                container_util::remove_duplicate(result);
                return result;
            }
            std::vector<OpenVolumeMesh::HalfFaceHandle> vertex_halffaces(OpenVolumeMesh::VertexHandle v) const {
                std::vector<OpenVolumeMesh::HalfFaceHandle> result;
                for (auto he : Base::outgoing_halfedges(v))
                    for (auto hf : Base::halfedge_halffaces(he))
                        result.push_back(hf);
                container_util::remove_duplicate(result);
                return result;
            }
            std::vector<OpenVolumeMesh::HalfFaceHandle> halffaces_incident_to_cell_corner(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::VertexHandle v) const {
                std::vector<OpenVolumeMesh::HalfFaceHandle> result;
                for (auto hf : Base::cell(c).halffaces()) {
                    bool is_incident = false;
                    for (auto w : Base::halfface_vertices(hf)) {
                        if (v == w) {
                            is_incident = true;
                            break;
                        }
                    }
                    if (is_incident)
                        result.push_back(hf);
                }
                return result;
            }
            std::vector<OpenVolumeMesh::HalfEdgeHandle> outgoing_halfedges_in_cell(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::VertexHandle v) const {
                std::vector<OpenVolumeMesh::HalfEdgeHandle> result;
                for (auto he : Base::outgoing_halfedges(v)) {
                    for (auto c2 : Base::halfedge_cells(he)) {
                        if (c == c2) {
                            result.push_back(he);
                            break;
                        }
                    }
                }
                return result;
            }
            // edge incidence
            std::vector<OpenVolumeMesh::CellHandle> edge_cells(OpenVolumeMesh::EdgeHandle e) const {
                std::vector<OpenVolumeMesh::CellHandle> result;
                auto he = Base::halfedge_handle(e, 0);
                for (auto c : Base::halfedge_cells(he))
                    result.push_back(c);
                return result;
            }
            std::vector<OpenVolumeMesh::FaceHandle> edge_faces(OpenVolumeMesh::EdgeHandle e) const {
                std::vector<OpenVolumeMesh::FaceHandle> result;
                auto he = Base::halfedge_handle(e, 0);
                for (auto hf : Base::halfedge_halffaces(he))
                    result.push_back(Base::face_handle(hf));
                return result;
            }
            std::pair<OpenVolumeMesh::VertexHandle, OpenVolumeMesh::VertexHandle> edge_vertices(OpenVolumeMesh::EdgeHandle e) const {
                auto he = Base::halfedge(Base::halfedge_handle(e, 0));
                return { he.from_vertex(), he.to_vertex() };
            }
            // face incidence
            std::vector<OpenVolumeMesh::EdgeHandle> face_edges(OpenVolumeMesh::FaceHandle f) const {
                std::vector<OpenVolumeMesh::EdgeHandle> result;
                for (auto he : Base::face(f).halfedges())
                    result.push_back(Base::edge_handle(he));
                return result;
            }
            std::vector<OpenVolumeMesh::VertexHandle> face_vertices(OpenVolumeMesh::FaceHandle f) const {
                std::vector<OpenVolumeMesh::VertexHandle> result;
                auto hf = Base::halfface_handle(f, 0);
                for (auto v : Base::halfface_vertices(hf))
                    result.push_back(v);
                return result;
            }
            // halfface incidence
            std::vector<OpenVolumeMesh::HalfEdgeHandle> halfface_halfedges(OpenVolumeMesh::HalfFaceHandle hf) const {
                auto face = Base::halfface(hf);
                std::vector<OpenVolumeMesh::HalfEdgeHandle> result;
                for (auto he : face.halfedges())
                    result.push_back(he);
                return result;
            }
            std::vector<OpenVolumeMesh::EdgeHandle> halfface_edges(OpenVolumeMesh::HalfFaceHandle hf) const {
                auto face = Base::halfface(hf);
                std::vector<OpenVolumeMesh::EdgeHandle> result;
                for (auto he : face.halfedges())
                    result.push_back(Base::edge_handle(he));
                return result;
            }
            // cell incidence
            std::vector<OpenVolumeMesh::EdgeHandle> cell_edges(OpenVolumeMesh::CellHandle c) const {
                std::vector<OpenVolumeMesh::EdgeHandle> result;
                for (auto hf : Base::cell(c).halffaces())
                    for (auto he : halfface_halfedges(hf))
                        result.push_back(Base::edge_handle(he));
                container_util::remove_duplicate(result);
                return result;
            }
            std::vector<OpenVolumeMesh::FaceHandle> cell_faces(OpenVolumeMesh::CellHandle c) const {
                std::vector<OpenVolumeMesh::FaceHandle> result;
                for (auto hf : Base::cell(c).halffaces())
                    result.push_back(Base::face_handle(hf));
                return result;
            }
            // incidence test ---------------------------
            // from halfedge
            bool is_incident(OpenVolumeMesh::HalfEdgeHandle he, OpenVolumeMesh::VertexHandle v) const {
                auto edge = Base::halfedge(he);
                return edge.from_vertex() == v || edge.to_vertex() == v;
            }
            // from edge
            bool is_incident(OpenVolumeMesh::EdgeHandle e, OpenVolumeMesh::VertexHandle v) const {
                auto edge = Base::edge(e);
                return edge.from_vertex() == v || edge.to_vertex() == v;
            }
            // from halfface
            bool is_incident(OpenVolumeMesh::HalfFaceHandle hf, OpenVolumeMesh::VertexHandle v) const {
                for (auto v2 : Base::halfface_vertices(hf))
                    if (v == v2)
                        return true;
                return false;
            }
            bool is_incident(OpenVolumeMesh::HalfFaceHandle hf, OpenVolumeMesh::HalfEdgeHandle he) const {
                for (auto he2 : halfface_halfedges(hf))
                    if (he == he2)
                        return true;
                return false;
            }
            bool is_incident(OpenVolumeMesh::HalfFaceHandle hf, OpenVolumeMesh::EdgeHandle e) const {
                for (auto e2 : halfface_edges(hf))
                    if (e == e2)
                        return true;
                return false;
            }
            // from face
            bool is_incident(OpenVolumeMesh::FaceHandle f, OpenVolumeMesh::VertexHandle v) const {
                auto hf = Base::halfface_handle(f, 0);
                return is_incident(hf, v);
            }
            bool is_incident(OpenVolumeMesh::FaceHandle f, OpenVolumeMesh::HalfEdgeHandle he) const {
                for (int i = 0; i < 2; ++i)
                    if (is_incident(Base::halfface_handle(f, i), he))
                        return true;
                return false;
            }
            bool is_incident(OpenVolumeMesh::FaceHandle f, OpenVolumeMesh::EdgeHandle e) const {
                return is_incident(f, Base::halfedge_handle(e, 0));
            }
            // from cell
            bool is_incident(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::VertexHandle v) const {
                for (auto v2 : Base::cell_vertices(c))
                    if (v == v2)
                        return true;
                return false;
            }
            bool is_incident(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::HalfEdgeHandle he) const {
                for (auto hf : Base::cell(c).halffaces()) {
                    auto face = Base::halfface(hf);
                    for (auto he2 : face.halfedges())
                        if (he == he2)
                            return true;
                }
                return false;
            }
            bool is_incident(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::EdgeHandle e) const {
                return is_incident(c, Base::halfedge_handle(e, 0));
            }
            bool is_incident(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::HalfFaceHandle hf) const {
                return c == Base::incident_cell(hf);
            }
            bool is_incident(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::FaceHandle f) const {
                for (int i = 0; i < 2; ++i)
                    if (is_incident(Base::halfface_handle(f, i)))
                        return true;
                return false;
            }
            // adjacency query -----------------
            // vertex-vertex adjacency: true if an edge exists
            bool is_adjacent(OpenVolumeMesh::VertexHandle v1, OpenVolumeMesh::VertexHandle v2) const {
                return Base::halfedge(v1, v2).is_valid();
            }
            // edge-edge adjacency: true if two edges share an endpoint
            bool is_adjacent(OpenVolumeMesh::EdgeHandle e1, OpenVolumeMesh::EdgeHandle e2) const {
                auto& edge1 = Base::edge(e1);
                auto& edge2 = Base::edge(e2);
                auto e1_from = edge1.from_vertex();
                auto e1_to = edge1.to_vertex();
                auto e2_from = edge2.from_vertex();
                auto e2_to = edge2.to_vertex();
                return e1_from == e2_from || e1_from == e2_to || e1_to == e2_from || e1_to == e2_to;
            }
            // edge-face adjacency: true if an edge and a face share one vertex
            bool is_adjacent(OpenVolumeMesh::EdgeHandle e, OpenVolumeMesh::FaceHandle f) const {
                for (auto e2 : face_edges(f))
                    if (is_adjacent(e, e2))
                        return true;
                return false;
            }
            // edge-cell adjacency: true if an edge and a cell share one vertex
            bool is_adjacent(OpenVolumeMesh::EdgeHandle e, OpenVolumeMesh::CellHandle c) const {
                for (auto e2 : cell_edges(c))
                    if (is_adjacent(e, e2))
                        return true;
                return false;
            }
            // face-face adjacency: true if two faces share one vertex
            bool is_adjacent_by_vertex(OpenVolumeMesh::FaceHandle f1, OpenVolumeMesh::FaceHandle f2) const {
                for (auto v1 : face_vertices(f1))
                    for (auto v2 : face_vertices(f2))
                        if (v1 == v2)
                            return true;
                return false;
            }
            // face-face adjacency: true if two faces share one edge
            bool is_adjacent_by_edge(OpenVolumeMesh::FaceHandle f1, OpenVolumeMesh::FaceHandle f2) const {
                for (auto e1 : face_edges(f1))
                    for (auto e2 : face_edges(f2))
                        if (e1 == e2)
                            return true;
                return false;
            }
            // face-cell adjacency: true if a face and a cell share one vertex
            bool is_adjacent_by_vertex(OpenVolumeMesh::FaceHandle f, OpenVolumeMesh::CellHandle c) const {
                for (auto v1 : face_vertices(f))
                    for (auto v2 : Base::cell_vertices(c))
                        if (v1 == v2)
                            return true;
                return false;
            }
            // face-cell adjacency: true if a face and a cell share one edge
            bool is_adjacent_by_edge(OpenVolumeMesh::FaceHandle f, OpenVolumeMesh::CellHandle c) const {
                for (auto e : face_edges(f))
                    for (auto c2 : Base::halfedge_cells(Base::halfedge_handle(e, 0)))
                        if (c == c2)
                            return true;
                return false;
            }
            // cell-cell adjacency: true if two cells share one vertex
            bool is_adjacent_by_vertex(OpenVolumeMesh::CellHandle c1, OpenVolumeMesh::CellHandle c2) const {
                for (auto v1 : Base::cell_vertices(c1))
                    for (auto v2 : Base::cell_vertices(c2))
                        if (v1 == v2)
                            return true;
                return false;
            }
            // cell-cell adjacency: true if two cells share one edge
            bool is_adjacent_by_edge(OpenVolumeMesh::CellHandle c1, OpenVolumeMesh::CellHandle c2) const {
                for (auto e1 : cell_edges(c1))
                    for (auto e2 : cell_edges(c2))
                        if (e1 == e2)
                            return true;
                return false;
            }
            // cell-cell adjacency: true if two cells share one face
            bool is_adjacent_by_face(OpenVolumeMesh::CellHandle c1, OpenVolumeMesh::CellHandle c2) const {
                for (auto c3 : Base::cell_cells(c1))
                    if (c2 == c3)
                        return true;
                return false;
            }
            // adjacent elements within cell
            std::pair<OpenVolumeMesh::HalfFaceHandle, OpenVolumeMesh::HalfFaceHandle> adjacent_halffaces_in_cell(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::EdgeHandle e) {
                OpenVolumeMesh::HalfFaceHandle hf0, hf1;
                for (auto hf : Base::cell(c).halffaces()) {
                    if (is_incident(hf, e)) {
                        if (hf0.is_valid()) {
                            hf1 = hf;
                            break;
                        } else {
                            hf0 = hf;
                        }
                    }
                }
                return { hf0, hf1 };
            }
            std::pair<OpenVolumeMesh::FaceHandle, OpenVolumeMesh::FaceHandle> adjacent_faces_in_cell(OpenVolumeMesh::CellHandle c, OpenVolumeMesh::EdgeHandle e) {
                auto p = adjacent_halffaces_in_cell(c, e);
                return { Base::face_handle(p.first), Base::face_handle(p.second) };
            }
        };
    }   // namespace openvolumemesh
}   // namespace kt84
