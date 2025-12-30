#pragma once
// Remember to define TETLIBRARY when building as a static library
#include <tetgen.h>
#include <vector>
#include <string>
#include <utility>

namespace kt84 {
    namespace tetgen_util {
        struct Format_node {
            std::vector<double> pointlist;
            std::vector<double> pointattributelist;
            std::vector<int   > pointmarkerlist;
        };
        struct Format_poly {
            Format_node node;
            struct Facet {
                std::vector<std::vector<int>> polygonlist;
                std::vector<double> holelist;
            };
            std::vector<Facet> facetlist;
            //std::vector<tetgenio::facet> facetlist;
            std::vector<int> facetmarkerlist;
            std::vector<double> holelist;
            // TODO: region list
        };
        struct Format_smesh {
            Format_node node;
            std::vector<std::vector<int>> facetlist;
            std::vector<int> facetmarkerlist;
            std::vector<double> holelist;
            // TODO: region list
            
            // conversion to .poly
            operator Format_poly() const {
                Format_poly poly;
                poly.node = node;
                poly.facetlist.reserve(facetlist.size());
                for (auto& f : facetlist)
                    poly.facetlist.push_back({{f}, {}});
                poly.facetmarkerlist = facetmarkerlist;
                poly.holelist = holelist;
                return poly;
            }
        };
        struct Format_off {
            std::vector<double> vertices;
            std::vector<std::vector<int>> faces;
            
            // conversion to .poly
            operator Format_poly() const {
                Format_poly poly;
                poly.node.pointlist = vertices;
                poly.facetlist.reserve(faces.size());
                for (auto& f : faces)
                    poly.facetlist.push_back({{f}, {}});
                return poly;
            }
        };
        struct Format_ele {
            int numberofcorners = 4;
            std::vector<int> tetrahedronlist;
            // TODO: tetrahedronattributelist: somewhat inconsistent descriptions in the manual/header
            // TODO: tetrahedronvolumelist
        };
        struct Format_face {
            std::vector<int> trifacelist;
            std::vector<int> trifacemarkerlist;
        };
        struct Format_edge {
            std::vector<int> edgelist;
            std::vector<int> edgemarkerlist;
        };
        struct Format_neigh {
            std::vector<int> neighborlist;
        };
        inline std::tuple<Format_node, Format_ele, Format_face, Format_edge, Format_neigh> tetrahedralize(const Format_poly& in_poly, const std::string& switches) {
            tetgenio in;
            //----+
            // IN |
            //----+
            // points
            in.numberofpoints = in_poly.node.pointlist.size() / 3;
            in.pointlist      = const_cast<double*>(&in_poly.node.pointlist[0]);
            // point attributes
            in.numberofpointattributes = in_poly.node.pointattributelist.size() / in.numberofpoints;
            if (!in_poly.node.pointattributelist.empty())
                in.pointattributelist = const_cast<double*>(&in_poly.node.pointattributelist[0]);
            // point boundary markers
            if (!in_poly.node.pointmarkerlist.empty())
                in.pointmarkerlist = const_cast<int*>(&in_poly.node.pointmarkerlist[0]);
            // faces
            in.numberoffacets = in_poly.facetlist.size();
            std::vector<tetgenio::facet> facetlist(in.numberoffacets);
            std::vector<std::vector<tetgenio::polygon>> facetpolygonlist(in.numberoffacets);
            in.facetlist = &facetlist[0];
            for (int i = 0; i < in.numberoffacets; ++i) {
                facetpolygonlist[i].reserve(in_poly.facetlist[i].polygonlist.size());
                for (auto& p : in_poly.facetlist[i].polygonlist)
                    facetpolygonlist[i].push_back({const_cast<int*>(&p[0]), static_cast<int>(p.size())});
                in.facetlist[i].numberofpolygons = facetpolygonlist[i].size();
                in.facetlist[i].polygonlist = &facetpolygonlist[i][0];
                in.facetlist[i].numberofholes = in_poly.facetlist[i].holelist.size() / 3;
                in.facetlist[i].holelist = in.facetlist[i].numberofholes ? const_cast<double*>(&in_poly.facetlist[i].holelist[0]) : nullptr;
            }
            // holes
            in.numberofholes = in_poly.holelist.size() / 3;
            if (!in_poly.holelist.empty())
                in.holelist = const_cast<double*>(&in_poly.holelist[0]);
            // TODO: regions
            
            tetgenio out;
            ::tetrahedralize(const_cast<char*>(switches.c_str()), &in, &out);
            
            //-----+
            // OUT |
            //-----+
            Format_node out_node;
            // pointlist
            out_node.pointlist.resize(3 * out.numberofpoints);
            for (size_t i = 0; i < out_node.pointlist.size(); ++i)
                out_node.pointlist[i] = out.pointlist[i];
            // pointattributelist
            out_node.pointattributelist.resize(out.numberofpointattributes * out.numberofpoints);
            for (size_t i = 0; i < out_node.pointattributelist.size(); ++i)
                out_node.pointattributelist[i] = out.pointattributelist[i];
            // pointmarkerlist
            if (out.pointmarkerlist != nullptr) {
                out_node.pointmarkerlist.resize(out.numberofpoints);
                for (size_t i = 0; i < out_node.pointmarkerlist.size(); ++i)
                    out_node.pointmarkerlist[i] = out.pointmarkerlist[i];
            }
            
            Format_ele out_ele;
            // tetrahedronlist
            out_ele.numberofcorners = out.numberofcorners;
            out_ele.tetrahedronlist.resize(out.numberofcorners * out.numberoftetrahedra);
            for (size_t i = 0; i < out_ele.tetrahedronlist.size(); ++i)
                out_ele.tetrahedronlist[i] = out.tetrahedronlist[i];
            // TODO: tetrahedronattributelist

            Format_face out_face;
            // trifacelist
            out_face.trifacelist.resize(3 * out.numberoftrifaces);
            for (size_t i = 0; i < out_face.trifacelist.size(); ++i)
                out_face.trifacelist[i] = out.trifacelist[i];
            // trifacemarkerlist
            if (out.trifacemarkerlist != nullptr) {
                out_face.trifacemarkerlist.resize(out.numberoftrifaces);
                for (int i = 0; i < out.numberoftrifaces; ++i)
                    out_face.trifacemarkerlist[i] = out.trifacemarkerlist[i];
            }
            
            Format_edge out_edge;
            // edgelist
            out_edge.edgelist.resize(2 * out.numberofedges);
            for (size_t i = 0; i < out_edge.edgelist.size(); ++i)
                out_edge.edgelist[i] = out.edgelist[i];
            // edgemarkerlist
            if (out.edgemarkerlist != nullptr) {
                out_edge.edgemarkerlist.resize(out.numberofedges);
                for (int i = 0; i < out.numberofedges; ++i)
                    out_edge.edgemarkerlist[i] = out.edgemarkerlist[i];
            }
            
            Format_neigh out_neigh;
            // neighborlist
            if (out.neighborlist != nullptr) {
                out_neigh.neighborlist.resize(4 * out.numberoftetrahedra);
                for (size_t i = 0; i < out_neigh.neighborlist.size(); ++i)
                    out_neigh.neighborlist[i] = out.neighborlist[i];
            }
            
            in.initialize();
            return std::make_tuple(out_node, out_ele, out_face, out_edge, out_neigh);
        }
    }
}
