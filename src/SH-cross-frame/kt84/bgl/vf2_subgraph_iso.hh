#pragma once
#include <map>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/vf2_sub_graph_iso.hpp>
#include "../util.hh"

namespace kt84 {
    namespace internal {
        template <class Graph>
        struct VF2SubgraphIsoCallback {
            VF2SubgraphIsoCallback(const Graph& graph_small, const Graph& graph_large, std::vector<std::map<int, int>>& result)
                : graph1(graph_small), graph2(graph_large), result(result)
            {}
            template <typename Map1To2, typename Map2To1>
            bool operator()(Map1To2 f, Map2To1 g) const {
                using namespace boost;
                std::map<int, int> m;
                BGL_FORALL_VERTICES_T(v, graph1, Graph)
                    m.insert(std::make_pair<int, int>(get(vertex_index_t(), graph1, v), get(vertex_index_t(), graph2, get(f, v))));
                result.push_back(m);
                return true;
            }
            const Graph& graph1;
            const Graph& graph2;
            std::vector<std::map<int, int>>& result;
        };
    }
    template <class DirectedS = boost::undirectedS>
    inline std::vector<std::map<int, int>> vf2_subgraph_iso(const std::vector<std::pair<int, int>>& graph_small_edges, const std::vector<std::pair<int, int>>& graph_large_edges) {
        using namespace boost;
        if (graph_small_edges.size() > graph_large_edges.size())
            return {};
        // determine num_vertices
        int num_vertices_small = 0;
        int num_vertices_large = 0;
        for (auto& e : graph_small_edges) num_vertices_small = std::max<int>(num_vertices_small, util::max<int>(e));
        for (auto& e : graph_large_edges) num_vertices_large = std::max<int>(num_vertices_large, util::max<int>(e));
        ++num_vertices_small;
        ++num_vertices_large;
        // construct boost graphs
        using Graph = adjacency_list<vecS, vecS, DirectedS>;
        Graph graph_small(graph_small_edges.begin(), graph_small_edges.end(), num_vertices_small);
        Graph graph_large(graph_large_edges.begin(), graph_large_edges.end(), num_vertices_large);
        // call boost::vf2_sub_graphiso with custom callback
        std::vector<std::map<int, int>> result;
        internal::VF2SubgraphIsoCallback<Graph> callback(graph_small, graph_large, result);
        vf2_subgraph_iso(graph_small, graph_large, callback);
        return result;
    }
}
