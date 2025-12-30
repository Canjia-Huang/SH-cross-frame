#pragma once
#include <vector>
#include <OpenVolumeMesh/Core/Iterators.hh>
#include <OpenVolumeMesh/Core/OpenVolumeMeshHandle.hh>

namespace kt84 {
    namespace openvolumemesh {
        struct StaticProperty_EmptyTraits {
            struct VertexData {};
            struct EdgeData {};
            struct FaceData {};
            struct CellData {};
            struct HalfEdgeData {};
            struct HalfFaceData {};
        };
        template <class Traits, class Base>
        struct StaticProperty : public Base {
            using VertexData = typename Traits::VertexData;
            using EdgeData = typename Traits::EdgeData;
            using FaceData = typename Traits::FaceData;
            using CellData = typename Traits::CellData;
            using HalfEdgeData = typename Traits::HalfEdgeData;
            using HalfFaceData = typename Traits::HalfFaceData;
            // data access
                  VertexData& data(const OpenVolumeMesh::VertexHandle& v)       { return internal_vertex_data[v.idx()]; }
            const VertexData& data(const OpenVolumeMesh::VertexHandle& v) const { return internal_vertex_data[v.idx()]; }
                  EdgeData& data(const OpenVolumeMesh::EdgeHandle& e)       { return internal_edge_data[e.idx()]; }
            const EdgeData& data(const OpenVolumeMesh::EdgeHandle& e) const { return internal_edge_data[e.idx()]; }
                  FaceData& data(const OpenVolumeMesh::FaceHandle& f)       { return internal_face_data[f.idx()]; }
            const FaceData& data(const OpenVolumeMesh::FaceHandle& f) const { return internal_face_data[f.idx()]; }
                  CellData& data(const OpenVolumeMesh::CellHandle& c)       { return internal_cell_data[c.idx()]; }
            const CellData& data(const OpenVolumeMesh::CellHandle& c) const { return internal_cell_data[c.idx()]; }
                  HalfEdgeData& data(const OpenVolumeMesh::HalfEdgeHandle& he)       { return internal_halfedge_data[he.idx()]; }
            const HalfEdgeData& data(const OpenVolumeMesh::HalfEdgeHandle& he) const { return internal_halfedge_data[he.idx()]; }
                  HalfFaceData& data(const OpenVolumeMesh::HalfFaceHandle& hf)       { return internal_halfface_data[hf.idx()]; }
            const HalfFaceData& data(const OpenVolumeMesh::HalfFaceHandle& hf) const { return internal_halfface_data[hf.idx()]; }
            // topology modification
            template <class VecT>
            OpenVolumeMesh::VertexHandle add_vertex(const VecT &p) {
                auto r = Base::add_vertex(p);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::VertexIter delete_vertex(const OpenVolumeMesh::VertexHandle &h) {
                auto r = Base::delete_vertex(h);
                internal_resize_data();
                return r;
            }
            void clear() {
                Base::clear();
                internal_vertex_data.clear();
                internal_edge_data.clear();
                internal_face_data.clear();
                internal_cell_data.clear();
                internal_halfedge_data.clear();
                internal_halfface_data.clear();
            }
            OpenVolumeMesh::FaceHandle add_face(const std::vector<OpenVolumeMesh::VertexHandle> &vertices) {
                auto r = Base::add_face(vertices);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::FaceHandle add_face(const std::vector<OpenVolumeMesh::HalfEdgeHandle> &halfedges, bool topologyCheck = false) {
                auto r = Base::add_face(halfedges, topologyCheck);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::CellHandle add_cell(const std::vector<OpenVolumeMesh::VertexHandle> &vertices, bool topologyCheck = false) {
                auto r = Base::add_cell(vertices, topologyCheck);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::CellHandle add_cell(const std::vector<OpenVolumeMesh::HalfFaceHandle> &halffaces, bool topologyCheck = false) {
                auto r = Base::add_cell(halffaces, topologyCheck);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::EdgeIter delete_edge(const OpenVolumeMesh::EdgeHandle &h) {
                auto r = Base::delete_edge(h);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::FaceIter delete_face(const OpenVolumeMesh::FaceHandle &h) {
                auto r = Base::delete_face(h);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::CellIter delete_cell(const OpenVolumeMesh::CellHandle &h) {
                auto r = Base::delete_cell(h);
                internal_resize_data();
                return r;
            }
            OpenVolumeMesh::CellIter delete_cell_range(const OpenVolumeMesh::CellIter &first, const OpenVolumeMesh::CellIter &last) {
                auto r = Base::delete_cell(first, last);
                internal_resize_data();
                return r;
            }
            // reserve
            void reserve_vertices(int size) { internal_vertex_data.reserve(size); }
            void reserve_edges(int size) {
                internal_edge_data.reserve(size);
                internal_halfedge_data.reserve(2 * size);
            }
            void reserve_faces(int size) {
                internal_face_data.reserve(size);
                internal_halfface_data.reserve(2 * size);
            }
            void reserve_cells(int size) { internal_cell_data.reserve(size); }
        private:
            std::vector<VertexData> internal_vertex_data;
            std::vector<EdgeData> internal_edge_data;
            std::vector<FaceData> internal_face_data;
            std::vector<CellData> internal_cell_data;
            std::vector<HalfEdgeData> internal_halfedge_data;
            std::vector<HalfFaceData> internal_halfface_data;
            void internal_resize_data() {
                internal_vertex_data.resize(Base::n_vertices());
                internal_edge_data.resize(Base::n_edges());
                internal_face_data.resize(Base::n_faces());
                internal_cell_data.resize(Base::n_cells());
                internal_halfedge_data.resize(Base::n_halfedges());
                internal_halfface_data.resize(Base::n_halffaces());
            }
        };
    }
}
