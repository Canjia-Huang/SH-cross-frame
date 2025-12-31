//
// Created by huangcanjia on 25-12-31.
//

#include <iostream>
#include <sstream>
#include <CLI/CLI.hpp>
#include <Eigen/Geometry>
#include <geogram/basic/command_line_args.h>
#include <geogram/mesh/mesh_geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_tetrahedralize.h>
#include <gtest/gtest.h>
#include <igl/read_triangle_mesh.h>
#include "SH-cross-frame/frame3d.hh"
#include "SH-cross-frame/kt84/util.hh"
#include "SH-cross-frame/kt84/vector_cast.hh"
#include "utils/log.h"
#include "utils/parse_filepath.h"
#include "utils/throw_error.h"

Eigen::Matrix3d getRotationMatrixZYZ(const double a, const double b, const double c) {
    Eigen::Quaterniond q = Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(b, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(c, Eigen::Vector3d::UnitZ());
    return q.toRotationMatrix();
}

void output_fra(
    frame3d::TetMesh mesh,
    const std::string& path
    ) {
    std::ofstream out(path);
    if (!out.is_open())
        THROW_RUNTIME_ERROR("cannot write to " + path);

    out << "FRA 1" << std::endl;
    out << mesh.n_vertices() << " 3 4" << std::endl;
    for (auto v : mesh.vertices()) {
        auto& zyz = mesh.data(v).zyz;
        const auto R = getRotationMatrixZYZ(zyz[0], zyz[1], zyz[2]);
        out << R(0, 0) << " " << R(1, 0) << " " << R(2, 0) << std::endl;
        out << R(0, 1) << " " << R(1, 1) << " " << R(2, 1) << std::endl;
        out << R(0, 2) << " " << R(1, 2) << " " << R(2, 2) << std::endl;
    }
    out.close();
}

void output_frame_visualization(
    frame3d::TetMesh mesh,
    const std::string& path
    ) {
    GEO::Mesh M;

    /* Create tet mesh vertices */
    {
        GEO::index_t new_v = M.vertices.create_vertices(mesh.n_vertices());
        for (auto v : mesh.vertices()) {
            auto p = mesh.vertex(v);
            M.vertices.point(new_v++) = GEO::vec3(p[0], p[1], p[2]);
        }
    }

    /* Create tet mesh edges */
    {
        std::vector<std::pair<GEO::index_t, GEO::index_t>> edges;
        for (auto e : mesh.edges())
            edges.emplace_back(mesh.edge(e).from_vertex().idx(), mesh.edge(e).to_vertex().idx());

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
    // const double diagonal_length = std::sqrt(pow(xyzmax[0]-xyzmin[0], 2) + pow(xyzmax[1]-xyzmin[1], 2) + pow(xyzmax[2]-xyzmin[2], 2));
    // LOG::DEBUG("diagonal_length: {}", diagonal_length);
    const double minimum_length = std::min({xyzmax[0]-xyzmin[0], xyzmax[1]-xyzmin[1], xyzmax[2]-xyzmin[2]});
    LOG::DEBUG("minimum_length: {}", minimum_length);

    /* Create cubes */
    {
        const double l = minimum_length * 0.1;

        GEO::index_t new_v = M.vertices.create_vertices(8*mesh.n_vertices()); // per vertex
        GEO::index_t new_c = M.cells.create_hexes(mesh.n_vertices()); // per vertex
        for (auto v : mesh.vertices()) {
            auto p = mesh.vertex(v);
            auto& zyz = mesh.data(v).zyz;
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

const std::string app_name = "MeshOffset";

int main(const int argc, char *argv[]) {
    GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
    GEO::CmdLine::import_arg_group("standard");
    GEO::CmdLine::import_arg_group("algo");
    spdlog::set_level(spdlog::level::trace);

    std::string input_mesh_path;
    std::string output_fra_path;
    bool output_visualization = false;
    double w_boundary = 200;

    /*== App ================================================================================================*/
    CLI::App app{app_name};
    argv = app.ensure_utf8(argv);

    app.add_option(
        "-i",
        input_mesh_path,
        "input triangle/tetrahedral mesh path (if the input is a triangle mesh, will be "
        "tetrahedralization by tetgen, and save to `output_path`+`_tet.mesh`)"
        )->check(CLI::ExistingFile)->required();

    app.add_option(
        "-o",
        output_fra_path,
        "output per vertex frame field file (.fra) path");

    app.add_option(
        "-w",
        w_boundary,
        "boundary weight (default: 200)");

    app.add_flag(
        "-v",
        output_visualization,
        "output visualization file (.geogram)");

    CLI11_PARSE(app, argc, argv);

    /* Parse */
    if (output_fra_path.empty())
        output_fra_path = "./" + get_filename(input_mesh_path) + ".fra";

    /* == Let's go! ================================================================================================ */
    GEO::Mesh M;
    GEO::mesh_load(input_mesh_path, M);
    LOG::DEBUG("load mesh: #V-{}, #F-{}, #C-{}", M.vertices.nb(), M.facets.nb(), M.cells.nb());

    if (M.cells.nb() == 0) {
        LOG::TRACE("Tetrahedralization");

        GEO::MeshTetrahedralizeParameters params;
        params.preprocess = true;
        params.refine = true;
        params.refine_quality = 1.0;
        GEO::mesh_tetrahedralize(M, params);

        LOG::DEBUG("tet mesh: #V-{}, #F-{}, #C-{}", M.vertices.nb(), M.facets.nb(), M.cells.nb());

        const std::string output_tet_mesh_path = get_parent_path(output_fra_path) + get_filename(output_fra_path) + "_tet.mesh";
        LOG::INFO("save tetrahedralization to {}", output_tet_mesh_path);
        GEO::mesh_save(M, output_tet_mesh_path);
    }

    struct {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
    } trimesh;
    frame3d::TetMesh mesh;

    {
        LOG::TRACE("convert to trimesh and tetmesh");

        mesh.clear();
        // copy vertices
        mesh.reserve_vertices((int)M.vertices.nb());
        for (const auto& v : M.vertices)
            mesh.add_vertex(kt84::vector_cast<3, OpenVolumeMesh::Vec3d, GEO::vec3>(M.vertices.point(v)));
        // copy tets
        mesh.reserve_cells((int)M.cells.nb());
        for (const auto& c : M.cells) {
            OpenVolumeMesh::VertexHandle v[4];
            for (int j = 0; j < 4; ++j)
                v[j].idx(M.cells.vertex(c, j));
            mesh.add_cell(v[0], v[1], v[2], v[3]);
        }

        mesh.precompute();
        LOG::INFO("tetmesh statistics:");
        LOG::INFO("vertices: {}", mesh.n_vertices());
        LOG::INFO("tets: {}", mesh.n_cells());
        LOG::INFO("faces: {}", mesh.n_faces());
        LOG::INFO("edges: {}", mesh.n_edges());

        // tet volumes statistics
        double vol_max = 0, vol_min = 1000, vol_avg = 0;
        for (auto tet : mesh.cells()) {
            double vol = mesh.data(tet).tetCellVolume;
            vol_max = std::max<double>(vol_max, vol);
            vol_min = std::min<double>(vol_min, vol);
            vol_avg += vol / mesh.n_cells();
        }
        LOG::INFO("min/max/avg tet cell volume: {}/{}/{}", vol_min, vol_max, vol_avg);
    }
    {
        LOG::TRACE("compute boundary field");
        mesh.compute_boundary_field();
    }
    {
        LOG::TRACE("compute field");
        mesh.compute_field(w_boundary);
    }
    {
        LOG::TRACE("output fra to {}", output_fra_path);
        output_fra(mesh, output_fra_path);
    }
    if (output_visualization) {
        const std::string output_vis_path = get_parent_path(output_fra_path) + get_filename(output_fra_path) + "_fra_vis.geogram";
        LOG::TRACE("output visualization to {}", output_vis_path);
        output_frame_visualization(mesh, output_vis_path);
    }

    LOG::INFO("Hello World!");
}