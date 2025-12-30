#pragma once
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include "../util.hh"

namespace kt84 {
    inline std::vector<int> connected_components(const std::vector<std::pair<int, int>>& graph_edges) {
        using namespace boost;
        int num_vertices = 0;
        for (auto& e : graph_edges) num_vertices = std::max<int>(num_vertices, util::max<int>(e));
        ++num_vertices;
        adjacency_list<vecS, vecS, undirectedS> g(graph_edges.begin(), graph_edges.end(), num_vertices);
        std::vector<int> component_id(num_vertices);
        connected_components(g, &component_id[0]);
        return component_id;
    }
}
