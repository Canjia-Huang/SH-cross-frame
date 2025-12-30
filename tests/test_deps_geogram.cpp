//
// Created by huangcanjia on 25-12-30.
//

#include <filesystem>
#include <geogram/mesh/mesh_io.h>
#include <gtest/gtest.h>

namespace
{
    TEST(DepsTest, geogram)
    {
        GEO::Mesh M;
        M.vertices.create_vertices(4);
        M.vertices.point(0) = GEO::vec3(0,0,0);
        M.vertices.point(1) = GEO::vec3(1,0,0);
        M.vertices.point(2) = GEO::vec3(0,1,0);
        M.vertices.point(3) = GEO::vec3(0,0,1);
        M.cells.create_tet(0,1,2,3);

        const std::string write_file_path = "test_tet.mesh";
        EXPECT_TRUE(GEO::mesh_save(M, write_file_path));
        EXPECT_TRUE(std::filesystem::exists(write_file_path));

        GEO::Mesh M_in;
        EXPECT_TRUE(GEO::mesh_load(write_file_path, M_in));
        EXPECT_EQ(M_in.vertices.nb(), 4);
        EXPECT_EQ(M_in.cells.nb(), 1);
    }
}