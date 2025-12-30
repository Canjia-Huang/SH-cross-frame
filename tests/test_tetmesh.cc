// =============================================================================
// This file contains code derived from the following source(s):
//   Original Repository: https://kenshi84.github.io/misc/frame3d.zip
//
// Copyright (c) Kenshi Takayama
// -----------------------------------------------------------------------------
// Modifications made by Canjia Huang on 2025-12-30:
//   - Use gtest
//   - Modify some headers
//   - Remove visualization
//   - Replace `cout`/`scout` -> spdlog LOG
// =============================================================================

#include <iostream>
#include <sstream>
#include <Eigen/Geometry>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_geometry.h>
#include <gtest/gtest.h>
#include <igl/read_triangle_mesh.h>
#include <igl/tetgen/tetrahedralize.h>
#include "SH-cross-frame/frame3d.hh"
#include "SH-cross-frame/kt84/util.hh"
#include "SH-cross-frame/kt84/vector_cast.hh"
#include "SH-cross-frame/kt84/graphics/DisplayList.hh"
#include "SH-cross-frame/kt84/graphics/graphics_util.hh"
#include "utils/log.h"
#include "utils.h"

using namespace std;
using namespace kt84;
using namespace kt84::graphics_util;
using namespace Eigen;

struct Globals {
    double w_boundary = 200;
    char tetgen_switches[64] = "pYq1.414";
    struct {
        MatrixXd V;
        MatrixXi F;
    } trimesh;
    frame3d::TetMesh mesh;
    struct {
        DisplayList edges;
        DisplayList cross;
        DisplayList cube;
#ifdef FRAME3D_FIXED_BOUNDARY
        DisplayList trimesh;
#endif
    } displist;
    struct {
        double fovy = 40;
        bool edges = true;
        bool frames = true;
        bool vizmode = true;
        double frame_scale = 0.01;
#ifdef FRAME3D_FIXED_BOUNDARY
        bool trimesh = true;
#endif
    } drawopt;
    struct {
        double threshold = 0;
        int axis = -1;
    } crosssection;
#ifdef FRAME3D_FIXED_BOUNDARY
    struct {
        OpenVolumeMesh::HalfFaceHandle hf;
        Vector3d plane_origin;
        Vector3d plane_normal;
        Vector3d endpoint;
    } draginfo;
#endif
};

Eigen::Matrix3d getRotationMatrixZYZ(double a, double b, double c) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ());
    return q.toRotationMatrix();
}

class TetMeshTest : public ::testing::Test {
    void SetUp() override {}

public:
    void read_trimesh(
        const std::string& path
        ) {
        if (!igl::read_triangle_mesh(path, g.trimesh.V, g.trimesh.F)) return;

        LOG::INFO("trimesh statistics:");
        LOG::INFO("vertices: {}", g.trimesh.V.rows());
        LOG::INFO("faces: {}", g.trimesh.F.rows());

        // recenter & rescale
        AlignedBox3d bbox;
        for (int i = 0; i < g.trimesh.V.rows(); ++i)
            bbox.extend(Vector3d(g.trimesh.V.row(i)));
        for (int i = 0; i < g.trimesh.V.rows(); ++i) {
            g.trimesh.V.row(i) -= bbox.center();
            g.trimesh.V.row(i) /= bbox.diagonal().norm();
        }

        tetrahedralize();
    }

    void tetrahedralize(
        ) {
        MatrixXd TV;
        MatrixXi TT, TF;
        if (igl::tetrahedralize(g.trimesh.V, g.trimesh.F, g.tetgen_switches, TV, TT, TF) != 0) return;
        // I HAVE NO IDEA WHY THIS MAKES A DIFFERENCE (when purely axis-aligned cube is input)!
        for (int i = 0; i < TV.rows(); ++i)
            TV.row(i) += Vector3d::Random() * 0.00000000001;

        g.mesh.clear();
        // copy vertices
        g.mesh.reserve_vertices((int)TV.rows());
        for (int i = 0; i < TV.rows(); ++i)
            g.mesh.add_vertex(vector_cast<3, OpenVolumeMesh::Vec3d, Vector3d>(TV.row(i)));
        // copy tets
        g.mesh.reserve_cells((int)TT.rows());
        for (int i = 0; i < TT.rows(); ++i) {
            OpenVolumeMesh::VertexHandle v[4];
            for (int j = 0; j < 4; ++j)
                v[j].idx(TT(i, j));
            g.mesh.add_cell(TT(i, 0), v[1], v[2], v[3]);
        }
#ifndef NDEBUG
        g.mesh.debugInfo_get();
#endif
        g.mesh.precompute();
        LOG::INFO("tetmesh statistics:");
        LOG::INFO("vertices: {}", g.mesh.n_vertices());
        LOG::INFO("tets: {}", g.mesh.n_cells());
        LOG::INFO("faces: {}", g.mesh.n_faces());
        LOG::INFO("edges: {}", g.mesh.n_edges());

        // tet volumes statistics
        double vol_max = 0, vol_min = 1000, vol_avg = 0;
        for (auto tet : g.mesh.cells()) {
            double vol = g.mesh.data(tet).tetCellVolume;
            vol_max = max<double>(vol_max, vol);
            vol_min = min<double>(vol_min, vol);
            vol_avg += vol / g.mesh.n_cells();
        }
        LOG::INFO("min/max/avg tet cell volume: {}/{}/{}", vol_min, vol_max, vol_avg);

        g.displist = {};
    }

    void output_frame_visualization(
        const std::string& path
        ) {
        GEO::Mesh M;

        /* Create tet mesh vertices */
        {
            GEO::index_t new_v = M.vertices.create_vertices(g.mesh.n_vertices());
            for (auto v : g.mesh.vertices()) {
                auto p = g.mesh.vertex(v);
                M.vertices.point(new_v++) = GEO::vec3(p[0], p[1], p[2]);
            }
        }

        /* Create tet mesh edges */
        {
            std::vector<std::pair<GEO::index_t, GEO::index_t>> edges;
            for (auto e : g.mesh.edges())
                edges.emplace_back(g.mesh.edge(e).from_vertex().idx(), g.mesh.edge(e).to_vertex().idx());

            GEO::index_t new_e = M.edges.create_edges(edges.size());
            for (const auto& [ev0, ev1] : edges) {
                M.edges.set_vertex(new_e, 0, ev0);
                M.edges.set_vertex(new_e, 1, ev1);
                ++new_e;
            }
        }

        /* Get diagonal length */
        double xyzmin[3], xyzmax[3];
        GEO::get_bbox(M, xyzmin, xyzmax);
        const double diagonal_length = std::sqrt(pow(xyzmax[0]-xyzmin[0], 2) + pow(xyzmax[1]-xyzmin[1], 2) + pow(xyzmax[2]-xyzmin[2], 2));
        LOG::DEBUG("diagonal_length: {}", diagonal_length);

        /* Create cubes */
        {
            const double l = diagonal_length * 0.01;

            GEO::index_t new_v = M.vertices.create_vertices(8*g.mesh.n_vertices()); // per vertex
            GEO::index_t new_c = M.cells.create_hexes(g.mesh.n_vertices()); // per vertex
            for (auto v : g.mesh.vertices()) {
                auto p = g.mesh.vertex(v);
                auto& zyz = g.mesh.data(v).zyz;
                const auto R = getRotationMatrixZYZ(zyz[0], zyz[1], zyz[2]);

                const GEO::vec3 center(p[0], p[1], p[2]);
                const GEO::vec3 v0(R(0, 0), R(1, 0), R(2, 0));
                const GEO::vec3 v1(R(0, 1), R(1, 1), R(2, 1));
                const GEO::vec3 v2(R(0, 2), R(1,2), R(2, 2));

                M.vertices.point(new_v + 0) = center + l * v0 + l * v1 + l * v2;
                M.vertices.point(new_v + 1) = center - l * v0 + l * v1 + l * v2;
                M.vertices.point(new_v + 2) = center - l * v0 - l * v1 + l * v2;
                M.vertices.point(new_v + 3) = center + l * v0 - l * v1 + l * v2;
                M.vertices.point(new_v + 4) = center + l * v0 + l * v1 - l * v2;
                M.vertices.point(new_v + 5) = center - l * v0 + l * v1 - l * v2;
                M.vertices.point(new_v + 6) = center - l * v0 - l * v1 - l * v2;
                M.vertices.point(new_v + 7) = center + l * v0 - l * v1 - l * v2;

                M.cells.set_vertex(new_c, 0, new_v + 0);
                M.cells.set_vertex(new_c, 1, new_v + 3);
                M.cells.set_vertex(new_c, 2, new_v + 1);
                M.cells.set_vertex(new_c, 3, new_v + 2);
                M.cells.set_vertex(new_c, 4, new_v + 4);
                M.cells.set_vertex(new_c, 5, new_v + 7);
                M.cells.set_vertex(new_c, 6, new_v + 5);
                M.cells.set_vertex(new_c, 7, new_v + 6);

                new_v += 8;
                ++new_c;
            }
        }

        GEO::mesh_save(M, path);
    }

    Globals g;
};

TEST_F(TetMeshTest, bone) {
    read_trimesh(std::string(TESTDATA_PATH) + "bone.obj");
    tetrahedralize();
    g.mesh.compute_boundary_field();
    g.mesh.compute_field(g.w_boundary);
    output_frame_visualization(get_current_test_name() + ".geogram");
}