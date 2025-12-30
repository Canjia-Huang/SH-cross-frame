// =============================================================================
// This file contains code derived from the following source(s):
//   Original Repository: https://kenshi84.github.io/misc/frame3d.zip
//
// Copyright (c) Kenshi Takayama
// -----------------------------------------------------------------------------
// Modifications made by Canjia Huang on 2025-12-30:
//   - Modify some headers
//   - Replace `cout` -> spdlog LOG
// =============================================================================

#include "frame3d.hh"
#include "alglib/optimization.h"
#include <Eigen/IterativeLinearSolvers>
#include "utils/log.h"
#ifdef FRAME3D_FIXED_BOUNDARY
#include "igl/comiso/nrosy.h"
#include "kt84/vector_cast.hh"
#endif

using namespace std;
using namespace Eigen;

namespace {
    frame3d::SHVector sh_target;
    using ZYZxSH = Matrix<double, 3, 9>;
    frame3d::SHMatrix deriv_Rz(double theta) {
        double c1 = -1 * std::sin(1 * theta), s1 = 1 * std::cos(1 * theta);
        double c2 = -2 * std::sin(2 * theta), s2 = 2 * std::cos(2 * theta);
        double c3 = -3 * std::sin(3 * theta), s3 = 3 * std::cos(3 * theta);
        double c4 = -4 * std::sin(4 * theta), s4 = 4 * std::cos(4 * theta);
        frame3d::SHMatrix r;
        r <<
            c4, 0, 0, 0, 0, 0, 0, 0, s4,
            0, c3, 0, 0, 0, 0, 0, s3, 0,
            0, 0, c2, 0, 0, 0, s2, 0, 0,
            0, 0, 0, c1, 0, s1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, -s1, 0, c1, 0, 0, 0,
            0, 0, -s2, 0, 0, 0, c2, 0, 0,
            0, -s3, 0, 0, 0, 0, 0, c3, 0,
            -s4, 0, 0, 0, 0, 0, 0, 0, c4;
        return r;
    }
    ZYZxSH deriv_zyz2sh(const frame3d::ZYZVector& zyz) {
        const double alpha = zyz[0], beta = zyz[1], gamma = zyz[2];
        const auto Rz_alpha = frame3d::get_Rz(alpha);
        const auto Rz_beta  = frame3d::get_Rz(beta);
        const auto Rz_gamma = frame3d::get_Rz(gamma);
        const auto deriv_Rz_alpha = deriv_Rz(alpha);
        const auto deriv_Rz_beta  = deriv_Rz(beta);
        const auto deriv_Rz_gamma = deriv_Rz(gamma);
        const auto h = frame3d::get_identity();
        const auto Rx90    = frame3d::get_Rx90();
        const auto Rx90inv = frame3d::get_Rx90inv();
        frame3d::SHVector deriv_sh_alpha = Rz_gamma * Rx90inv * Rz_beta * Rx90 * deriv_Rz_alpha * h;    // WARNING: Using 'auto' here will cause VC compiler bug only in Release mode!
        frame3d::SHVector deriv_sh_beta  = Rz_gamma * Rx90inv * deriv_Rz_beta * Rx90 * Rz_alpha * h;
        frame3d::SHVector deriv_sh_gamma = deriv_Rz_gamma * Rx90inv * Rz_beta * Rx90 * Rz_alpha * h;
        ZYZxSH deriv_sh;
        deriv_sh <<
            deriv_sh_alpha.transpose(),
            deriv_sh_beta .transpose(),
            deriv_sh_gamma.transpose();
        return deriv_sh;
    }
    void function1_grad(const alglib::real_1d_array &x, double &func, alglib::real_1d_array &grad, void *ptr) {
        frame3d::ZYZVector zyz = { x[0], x[1], x[2] };
        auto sh = frame3d::zyz2sh(zyz);
        frame3d::SHVector sh_diff = sh - sh_target;
        func = sh_diff.squaredNorm();
        frame3d::ZYZVector deriv = 2 * deriv_zyz2sh(zyz) * sh_diff;
        grad[0] = deriv[0];
        grad[1] = deriv[1];
        grad[2] = deriv[2];
    }
    double robust_atan2(double y, double x) {
        if (x*x + y*y < 0.000000000001) {
            LOG::WARN("robust_atan2: almost invalid arguments!");
            return 0.0;
        }
        return atan2(y, x);
    }
    double robust_acos(double x) {
        return acos(min(max(x, -1.), 1.));
    }
    frame3d::ZYZVector rotm2eul(const Matrix3d& A) {
        /*
        http://staff.city.ac.uk/~sbbh653/publications/euler.pdf
                  |  cos(t), 0, sin(t) |
        RotY(t) = |    0   , 1,   0    |
                  | -sin(t), 0, cos(t) |

                  | cos(t), -sin(t), 0 |
        RotZ(t) = | sin(t),  cos(t), 0 |
                  |   0   ,    0   , 1 |
        
        A = RotZ(gamma) * RotY(beta) * RotZ(alpha)
        
        A00 = cos(alpha)*cos(beta)*cos(gamma) - sin(alpha)*sin(gamma)
        A01 = -cos(beta)*cos(gamma)*sin(alpha) - cos(alpha)*sin(gamma)
        A02 = cos(gamma)*sin(beta)
        A10 = cos(alpha)*cos(beta)*sin(gamma) + cos(gamma)*sin(alpha)
        A11 = -cos(beta)*sin(alpha)*sin(gamma) + cos(alpha)*cos(gamma)
        A12 = sin(beta)*sin(gamma)
        A20 = -cos(alpha)*sin(beta)
        A21 = sin(alpha)*sin(beta)
        A22 = cos(beta)
        */
        double alpha;
        double beta = robust_acos(A(2,2));
        double gamma;
        if (sin(beta) < 0.000001) {
            if (A(2,2) > 0) {
                /*
                beta = 0
                A00 = cos(alpha)*cos(gamma) - sin(alpha)*sin(gamma) = cos(alpha+gamma)
                A01 = -cos(gamma)*sin(alpha) - cos(alpha)*sin(gamma) = -sin(alpha+gamma)
                alpha+gamma = atan2(-A01, A00)
                */
                alpha = 0;
                gamma = robust_atan2(-A(0,1), A(0,0));
            } else {
                /*
                beta = pi
                A00 = -cos(alpha)*cos(gamma) - sin(alpha)*sin(gamma) = -cos(alpha-gamma)
                A01 = cos(gamma)*sin(alpha) - cos(alpha)*sin(gamma) = sin(alpha-gamma)
                alpha-gamma = atan2(A01, -A00)
                */
                alpha = 0;
                gamma = -robust_atan2(A(0,1), -A(0,0));
            }
        } else {
            alpha = robust_atan2(A(2,1), -A(2,0));
            gamma = robust_atan2(A(1,2), A(0,2));
        }
        return { alpha, beta, gamma };
    }
}

frame3d::ZYZVector frame3d::sh2zyz(const SHVector& sh) {
    sh_target = sh;
    alglib::real_1d_array x = "[0, 0, 0]";
    alglib::mincgstate state;
    alglib::mincgreport rep;
    alglib::mincgcreate(x, state);
    alglib::mincgoptimize(state, function1_grad);
    alglib::mincgresults(state, x, rep);
    return { x[0], x[1], x[2] };
}

void frame3d::TetMesh::precompute() {
    LOG::TRACE("calculating tet volumes and tet face areas...");
    tetFaceArea_compute();
    tetCellVolume_compute();
    tetFaceNormal_compute();
    totalVolume = 0;
    for (auto tet : cells())
        totalVolume += data(tet).tetCellVolume;
    surfaceArea = 0;
    for (auto f : faces())
        if (is_boundary(f))
            surfaceArea += data(f).tetFaceArea;
    
#ifdef FRAME3D_FIXED_BOUNDARY
    LOG::TRACE("extracting boundary trimesh...");
    // copy all vertices (interior vertices are kept redundantly)
    trimesh.V.resize(n_vertices(), 3);
    for (auto v : vertices()) {
        auto& p = vertex(v);
        trimesh.V.row(v.idx()) << p[0], p[1], p[2];
    }
    // collect boundary triangles
    trimesh.F.resize(0, 0);
    for (auto f : faces()) {
        if (!is_boundary(f)) continue;
        trimesh.F.conservativeResize(trimesh.F.rows() + 1, 3);
        int i = 0;
        for (auto v : face_vertices(f))
            trimesh.F(trimesh.F.rows() - 1, i++) = v.idx();
        data(f).idx_into_trimesh_F = trimesh.F.rows() - 1;
    }
#endif
    
    LOG::TRACE("constructing Laplacian matrix...");
    std::vector<Triplet<double>> triplets;
    int num_variables =
#ifdef FRAME3D_FIXED_BOUNDARY
        n_vertices();
#else
        n_vertices() * 9;     // 9 SH coeffs per vertex
#endif
    
    for (auto OABC : cells()) {
        // http://math.stackexchange.com/questions/49330
        for (auto OA : cell_edges(OABC)) {
            // figure out tet configuration
            auto BC = opposite_edge_handle_in_cell(OABC, OA);
            OpenVolumeMesh::EdgeHandle OB, OC, AB, CA;
            for (auto e1 : cell_edges(OABC)) {
                if (e1 == OA || e1 == BC) continue;
                OB = e1;
                CA = opposite_edge_handle_in_cell(OABC, OB);
                for (auto e2 : cell_edges(OABC)) {
                    if (e2 == OA || e2 == BC || e2 == OB || e2 == CA) continue;
                    OC = e2;
                    AB = opposite_edge_handle_in_cell(OABC, OC);
                    break;
                }
                break;
            }
            double a = data(OA).edgeLength, a2 = a * a;
            double b = data(OB).edgeLength, b2 = b * b;
            double c = data(OC).edgeLength, c2 = c * c;
            double d = data(BC).edgeLength, d2 = d * d;
            double e = data(CA).edgeLength, e2 = e * e;
            double f = data(AB).edgeLength, f2 = f * f;
            // face areas
            auto WX = adjacent_faces_in_cell(OABC, BC);
            auto YZ = adjacent_faces_in_cell(OABC, OA);
            double W = data(WX.first ).tetFaceArea;
            double X = data(WX.second).tetFaceArea;
            double Y = data(YZ.first ).tetFaceArea;
            double Z = data(YZ.second).tetFaceArea;
            // law of cosine
            double H2 = 1 / 16. * (4 * a2 * d2 - (b2 + e2 - c2 - f2) * (b2 + e2 - c2 - f2));
            double cos_OA = (Y*Y + Z*Z - H2) / (2 * Y * Z);
            // law of sine ( http://mathworld.wolfram.com/Tetrahedron.html )
            double V = data(OABC).tetCellVolume;
            double sin_OA = 3 * V * d / (2 * Y * Z);
            // accumulate cotangent coeff
            double coeff = d / 6 * cos_OA / sin_OA;
            int i, j;
            tie(i, j) = edge_vertices(BC);
#ifdef FRAME3D_FIXED_BOUNDARY
            triplets.push_back( { i, j, coeff } );
            triplets.push_back( { j, i, coeff } );
            triplets.push_back( { i, i, -coeff } );
            triplets.push_back( { j, j, -coeff } );
#else
            for (int k = 0; k < 9; ++k) {
                triplets.push_back( { 9 * i + k, 9 * j + k, coeff } );
                triplets.push_back( { 9 * j + k, 9 * i + k, coeff } );
                triplets.push_back( { 9 * i + k, 9 * i + k, -coeff } );
                triplets.push_back( { 9 * j + k, 9 * j + k, -coeff } );
            }
#endif
        }
    }
    SparseMatrix<double> L(num_variables, num_variables);
    L.setFromTriplets(triplets.begin(), triplets.end());
    LL = L * L;
}

#ifdef FRAME3D_FIXED_BOUNDARY
void frame3d::TetMesh::compute_boundary_field() {
    VectorXi b;
    MatrixXd bc;
    for (auto f : faces()) {
        if (!is_boundary(f) || !data(f).is_constrained) continue;
        b.conservativeResize(b.rows() + 1);
        bc.conservativeResize(bc.rows() + 1, 3);
        b[b.rows() - 1] = data(f).idx_into_trimesh_F;
        bc.row(bc.rows() - 1) = data(f).constraint_value;
    }
    MatrixXd R;
    VectorXd S;
    igl::nrosy(trimesh.V, trimesh.F, b, bc, 4, R, S);
    
    // initialize weighted sum of per-vertex SH vector
    for (auto v : vertices()) {
        data(v).sh.setZero();
        data(v).weight_sum = 0;
    }
    // weighted sum of per-face SH vector (obtained by converting 3x3 rotation matrix to ZYZ to SH) onto vertices
    for (auto hf : halffaces()) {
        if (!is_boundary(hf)) continue;
        auto f = face_handle(hf);
        Vector3d b1 = R.row(data(f).idx_into_trimesh_F);
        Vector3d n = kt84::vector_cast<3, Vector3d>(data(hf).tetFaceNormal);
        Vector3d b2 = n.cross(b1);
        Matrix3d A;
        A << b1, b2, n;
        auto zyz = rotm2eul(A);
        auto sh = zyz2sh(zyz);
        double area = data(f).tetFaceArea;
        for (auto v : face_vertices(f)) {
            data(v).sh += area * sh;
            data(v).weight_sum += area;
        }
    }
    for (auto v : vertices()) {
        if (!is_boundary(v)) continue;
        data(v).sh /= data(v).weight_sum;
        data(v).zyz = project(data(v).sh);
    }
}
#endif

void frame3d::TetMesh::compute_field(double w_boundary) {
    LOG::TRACE("assembling constraint matrix and right-hand-side...");
    std::vector<Triplet<double>> triplets;
    int num_constraints = 0;

#ifdef FRAME3D_FIXED_BOUNDARY
    int num_variables = n_vertices();
    MatrixXd b_constraints, b, x;
    for (auto v : vertices()) {
        if (!is_boundary(v)) continue;
        b_constraints.conservativeResize(++num_constraints, 9);
        double weight = data(v).weight_sum;
        auto& sh = data(v).sh;
        for (int i = 0; i < 9; ++i)
            b_constraints(num_constraints - 1, i) = weight * sh[i];
        triplets.push_back({ num_constraints - 1, v.idx(), weight });
    }

#else
    int num_variables = n_vertices() * 9;     // 9 SH coeffs per vertex
    VectorXd b_constraints, b, x;
    for (auto hf : halffaces()) {
        if (!is_boundary(hf)) continue;
        auto normal = data(hf).tetFaceNormal;
        double area = data(face_handle(hf)).tetFaceArea;
        double alpha = -robust_atan2(normal[1], normal[0]),             // eq.9
               beta  = -robust_acos(normal[2]),
               gamma = 0;
        SHMatrix R_n2z = get_Rz(gamma) * get_Rx90inv() * get_Rz(beta) * get_Rx90() * get_Rz(alpha);
        for (auto v : halfface_vertices(hf)) {
            b_constraints.conservativeResize(++num_constraints);
            b_constraints[num_constraints - 1] = area * sqrt(7);        // eq.13
            for (int i = 0; i < 9; ++i)
                triplets.push_back({ num_constraints - 1, 9 * v.idx() + i, area * R_n2z(4, i) });
        }
    }
#endif
    
    SparseMatrix<double> C(num_constraints, num_variables);
    C.setFromTriplets(triplets.begin(), triplets.end());
    SparseMatrix<double> Ct = C.transpose();
    SparseMatrix<double> A = (w_boundary / surfaceArea / surfaceArea) * Ct * C;
    A += (1. / std::cbrt(totalVolume)) * LL;
    b = (w_boundary / surfaceArea / surfaceArea) * Ct * b_constraints;
    
    LOG::TRACE("solving...");
    //SimplicialCholesky<SparseMatrix<double>> solver;
    //BiCGSTAB<SparseMatrix<double>> solver;
    ConjugateGradient<SparseMatrix<double>> solver;
    solver.compute(A);
    x = solver.solve(b);
    
    LOG::TRACE("converting to zyz...");
    for (auto v : vertices()) {
        auto& vdata = data(v);
        vdata.sh =
#ifdef FRAME3D_FIXED_BOUNDARY
            x.row(v.idx());
#else
            x.block<9, 1>(9 * v.idx(), 0);
#endif
        vdata.zyz = sh2zyz(vdata.sh);
    }
}
