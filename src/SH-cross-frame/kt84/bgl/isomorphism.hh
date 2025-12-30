#pragma once
#include <map>
#include <vector>
#include <utility>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/isomorphism.hpp>
#include "../util.hh"

namespace kt84 {
    /*
        params:
            const std::vector<std::pair<int, int>>& graph0_edges
                - Pairs of vertices defining graph0.
            const std::vector<std::pair<int, int>>& graph1_edges
                - Pairs of vertices defining graph1.
        retval:
            std::map<int, int> f
                - A map from a vertex v0 in graph0 to a vertex v1 in graph1, i.e., v1 = f[v0], satisfying 
                (v0,w0) \in graph0 <==> (f[v0],f[w0]) \in graph1.
    */
    template <class DirectedS = boost::undirectedS>
    inline std::map<int, int> isomorphism(const std::vector<std::pair<int, int>>& graph0_edges, const std::vector<std::pair<int, int>>& graph1_edges) {
        assert(graph0_edges.size() == graph1_edges.size());
        
        // determine maximum vertex index in the graphs
        int num_vertices = 0;
        for (auto& e0 : graph0_edges) num_vertices = std::max<int>(num_vertices, util::max<int>(e0));
        for (auto& e1 : graph1_edges) num_vertices = std::max<int>(num_vertices, util::max<int>(e1));
        ++num_vertices;
        
        // construct boost graphs
        using namespace boost;
        using Graph = adjacency_list<vecS, vecS, DirectedS>;
        Graph graph0(graph0_edges.begin(), graph0_edges.end(), num_vertices);
        Graph graph1(graph1_edges.begin(), graph1_edges.end(), num_vertices);
        
        // call boost::isomorphism
        std::vector<int> f_vec(num_vertices, -1);
        if (!isomorphism(graph0, graph1, isomorphism_map(make_iterator_property_map(f_vec.begin(), get(vertex_index, graph0)))))
            return std::map<int, int>();
            //return {};            // bullshit that std::map (also std::set and maybe more) in Xcode toolchain is not conforming to C++14 standard!
        
        // convert from std::vector to std::map
        std::map<int, int> f;
        for (auto& e0 : graph0_edges) {
            int v0[2] = { e0.first, e0.second };
            for (int i = 0; i < 2; ++i) {
                if (f.find(v0[i]) != f.end()) continue;
                f[v0[i]] = f_vec[v0[i]];
            }
        }
        
        return f;
    }
    // OpenMesh adaptor
    template <class Mesh0, class Mesh1>
    inline std::map<int, int> isomorphism(const Mesh0& mesh0, const Mesh1& mesh1) {
        std::map<int, int> failure;
        if (mesh0.n_vertices() != mesh1.n_vertices()) return failure;
        if (mesh0.n_faces() != mesh1.n_faces()) return failure;
        if (mesh0.n_edges() != mesh1.n_edges()) return failure;
        std::vector<std::pair<int, int>> mesh0_edges, mesh1_edges;
        for (auto e : mesh0.edges()) {
            std::pair<int, int> ev;
            ev.first  = mesh0.to_vertex_handle(mesh0.halfedge_handle(e, 0)).idx();
            ev.second = mesh0.to_vertex_handle(mesh0.halfedge_handle(e, 1)).idx();
            mesh0_edges.push_back(ev);
        }
        for (auto e : mesh1.edges()) {
            std::pair<int, int> ev;
            ev.first  = mesh1.to_vertex_handle(mesh1.halfedge_handle(e, 0)).idx();
            ev.second = mesh1.to_vertex_handle(mesh1.halfedge_handle(e, 1)).idx();
            mesh1_edges.push_back(ev);
        }
        return isomorphism(mesh0_edges, mesh1_edges);
    }
}
