// =============================================================================
// This file contains code derived from the following source(s):
//   Original Repository: https://kenshi84.github.io/misc/frame3d.zip
//
// Copyright (c) Kenshi Takayama
// -----------------------------------------------------------------------------
// Modifications made by Canjia Huang on 2025-12-30:
//   - Modify some headers
// =============================================================================

#pragma once
#include <cmath>
#include <Eigen/Sparse>
#include "OpenVolumeMesh/Mesh/PolyhedralMesh.hh"
#include "kt84/openvolumemesh/base/TetrahedralMesh.hh"
#include "kt84/openvolumemesh/base/Utility.hh"
#ifdef NDEBUG
#include "kt84/openvolumemesh/base/StaticProperty.hh"
#else
#include "kt84/openvolumemesh/base/DebugInfo.hh"
#endif
#include "kt84/openvolumemesh/base/TetFaceArea.hh"
#include "kt84/openvolumemesh/base/TetFaceNormal.hh"
#include "kt84/openvolumemesh/base/TetCellVolume.hh"

#define FRAME3D_FIXED_BOUNDARY

namespace frame3d {
    using SHVector = Eigen::Matrix<double, 9, 1>;
    using SHMatrix = Eigen::Matrix<double, 9, 9>;
    using ZYZVector = Eigen::Vector3d;
    inline SHMatrix get_Rz(double theta) {
        double c1 = std::cos(1 * theta), s1 = std::sin(1 * theta);
        double c2 = std::cos(2 * theta), s2 = std::sin(2 * theta);
        double c3 = std::cos(3 * theta), s3 = std::sin(3 * theta);
        double c4 = std::cos(4 * theta), s4 = std::sin(4 * theta);
        SHMatrix r;
        r <<
            c4, 0, 0, 0, 0, 0, 0, 0, s4,
            0, c3, 0, 0, 0, 0, 0, s3, 0,
            0, 0, c2, 0, 0, 0, s2, 0, 0,
            0, 0, 0, c1, 0, s1, 0, 0, 0,
            0, 0, 0, 0, 1, 0, 0, 0, 0,
            0, 0, 0, -s1, 0, c1, 0, 0, 0,
            0, 0, -s2, 0, 0, 0, c2, 0, 0,
            0, -s3, 0, 0, 0, 0, 0, c3, 0,
            -s4, 0, 0, 0, 0, 0, 0, 0, c4;
        return r;
    }
    inline SHMatrix get_Rx90() {
        using std::sqrt;
        SHMatrix r;
        r <<
            0          , 0        , 0          , 0        , 0         , sqrt(14)/4, 0         , -sqrt(2)/4, 0         ,
            0          , -3./4    , 0          , sqrt(7)/4, 0         , 0         , 0         , 0         , 0         ,
            0          , 0        , 0          , 0        , 0         , sqrt(2)/4 , 0         , sqrt(14)/4, 0         ,
            0          , sqrt(7)/4, 0          , 3./4     , 0         , 0         , 0         , 0         , 0         ,
            0          , 0        , 0          , 0        , 3./8      , 0         , sqrt(5)/4 , 0         , sqrt(35)/8,
            -sqrt(14)/4, 0        , -sqrt(2)/4 , 0        , 0         , 0         , 0         , 0         , 0         ,
            0          , 0        , 0          , 0        , sqrt(5)/4 , 0         , 1./2      , 0         , -sqrt(7)/4,
            sqrt(2)/4  , 0        , -sqrt(14)/4, 0        , 0         , 0         , 0         , 0         , 0         ,
            0          , 0        , 0          , 0        , sqrt(35)/8, 0         , -sqrt(7)/4, 0         , 1./8      ;
        return r;
    }
    inline SHMatrix get_Rx90inv() {
        using std::sqrt;
        SHMatrix r;
        r <<
            0         , 0        , 0         , 0        , 0         , -sqrt(14)/4, 0         , sqrt(2)/4  , 0         ,
            0         , -3./4    , 0         , sqrt(7)/4, 0         , 0          , 0         , 0          , 0         ,
            0         , 0        , 0         , 0        , 0         , -sqrt(2)/4 , 0         , -sqrt(14)/4, 0         ,
            0         , sqrt(7)/4, 0         , 3./4     , 0         , 0          , 0         , 0          , 0         ,
            0         , 0        , 0         , 0        , 3./8      , 0          , sqrt(5)/4 , 0          , sqrt(35)/8,
            sqrt(14)/4, 0        , sqrt(2)/4 , 0        , 0         , 0          , 0         , 0          , 0         ,
            0         , 0        , 0         , 0        , sqrt(5)/4 , 0          , 1./2      , 0          , -sqrt(7)/4,
            -sqrt(2)/4, 0        , sqrt(14)/4, 0        , 0         , 0          , 0         , 0          , 0         ,
            0         , 0        , 0         , 0        , sqrt(35)/8, 0          , -sqrt(7)/4, 0          , 1./8      ;
        return r;
    }
    inline SHVector get_identity() {
        SHVector h;
        h << 0, 0, 0, 0, std::sqrt(7), 0, 0, 0, std::sqrt(5);
        return h;
    }
    inline SHVector zyz2sh(const ZYZVector& zyz) {
        return get_Rz(zyz[2]) * get_Rx90inv() * get_Rz(zyz[1]) * get_Rx90() * get_Rz(zyz[0]) * get_identity();
    }
    ZYZVector sh2zyz(const SHVector& sh);
    inline ZYZVector project(SHVector& sh) {
        auto zyz = sh2zyz(sh);
        sh = zyz2sh(zyz);
        return zyz;
    }
    
    // 3d frame field over tetmesh
    struct TetMeshTraits : public kt84::openvolumemesh::StaticProperty_EmptyTraits {
        struct VertexData {
            SHVector sh;
            ZYZVector zyz;
#ifdef FRAME3D_FIXED_BOUNDARY
            double weight_sum = 0;
#endif
        };
        struct EdgeData : public kt84::openvolumemesh::EdgeLength_EdgeTraits {};
        struct FaceData : public kt84::openvolumemesh::TetFaceArea_FaceTraits {
#ifdef FRAME3D_FIXED_BOUNDARY
            Eigen::Vector3d constraint_value;     // one representative vector of 4 vectors in 4-RoSy
            bool is_constrained = false;
            int idx_into_trimesh_F = -1;
#endif
        };
        struct HalfFaceData : public kt84::openvolumemesh::TetFaceNormal_HalfFaceTraits {};
        struct CellData : public kt84::openvolumemesh::TetCellVolume_CellTraits {};
    };
    using TetMeshBase =
        kt84::openvolumemesh::TetrahedralMesh<
        kt84::openvolumemesh::TetCellVolume<
        kt84::openvolumemesh::TetFaceArea<
        kt84::openvolumemesh::TetFaceNormal<
#ifdef NDEBUG
        kt84::openvolumemesh::StaticProperty<TetMeshTraits,
#else
        kt84::openvolumemesh::DebugInfo<TetMeshTraits,
#endif
        kt84::openvolumemesh::Utility<
        OpenVolumeMesh::GeometricPolyhedralMeshV3d>>>>>>;
    struct TetMesh : public TetMeshBase {
        double totalVolume = 0;
        double surfaceArea = 0;
        Eigen::SparseMatrix<double> LL;
#ifdef FRAME3D_FIXED_BOUNDARY
        struct BoundaryTriMesh {
            Eigen::MatrixXd V;
            Eigen::MatrixXi F;
        } trimesh;
#endif
        void precompute();
#ifdef FRAME3D_FIXED_BOUNDARY
        void compute_boundary_field();
#endif
        void compute_field(double w_boundary);
    };
}
