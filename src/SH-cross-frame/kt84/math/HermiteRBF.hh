#pragma once

#include "PolynomialBasisGen.hh"
#include "RBFKernel.hh"
#include "FiniteDifferentiator.hh"
#include <Eigen/Dense>
#include <tuple>
#include <vector>

namespace kt84 {

template <int _DimIn, int _DimOut, class _RBFKernel_Core, int _DegreePolynomial>
struct HermiteRBF
    : public FiniteDifferentiator<HermiteRBF<_DimIn, _DimOut, _RBFKernel_Core, _DegreePolynomial>, _DimIn, _DimOut>
{
    enum {
        DimIn  = _DimIn,
        DimOut = _DimOut,
        DegreePolynomial = _DegreePolynomial,
    };
    
    typedef Eigen::Matrix<double, DimIn , 1> Point;
    typedef Eigen::Matrix<double, DimOut, 1> Value;
    typedef Eigen::Matrix<double, DimOut, DimIn> Gradient;
    typedef PolynomialBasisGenT<DimIn, DegreePolynomial> PolynomialBasisGen;
    typedef RBFKernel_Bivariate<DimIn, _RBFKernel_Core> Kernel;
    typedef std::tuple<Point, Value, Gradient> Constraint;
    
    static_assert(Kernel::GradientAlwaysDefined, "the chosen Kernel doesn't satisfy the requirement!");
    
    std::vector<Constraint> constraints;
    Kernel kernel;
    Eigen::Matrix<double, -1, DimOut> weights;
    Eigen::MatrixXd A_matrix;
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> A_factorized;
    
    void clear_constraints() {
        constraints.clear();
    }
    void add_constraint(const Point& point, const Value& value, const Gradient& gradient) {
        constraints.push_back(Constraint(point, value, gradient));
    }
    void factorize() {
        const int P = PolynomialBasisGen::DimOut;
        const int D = 1 + DimIn;
        const size_t n = constraints.size();
        const int m = D * n + P;
        A_matrix = Eigen::MatrixXd::Zero(m, m);
        for (size_t i = 0; i < n; ++i) {
            const Point& point_i = std::get<0>(constraints[i]);
            // rbf part
            for (size_t j = 0; j < n; ++j) {       // note that A is not symmetric
                const Point& point_j = std::get<0>(constraints[j]);
                // make submatrix
                Eigen::Matrix<double, D, D> A_sub;
                A_sub.setZero();
                A_sub(0, 0) = kernel(point_i, point_j);
                A_sub.template block<1, DimIn>(0, 1) = kernel.gradient(point_i, point_j);
                A_sub.template block<DimIn, 1>(1, 0) = A_sub.template block<1, DimIn>(0, 1).transpose();
                A_sub.template block<DimIn, DimIn>(1, 1) = kernel.hessian(point_i, point_j);
                // insert submatrix
                A_matrix.block<D, D>(D * i, D * j) = A_sub;
            }
            // polynomial part
            A_matrix.template block<P, D>(D * n, D * i) <<
                PolynomialBasisGen::basis(point_i),
                PolynomialBasisGen::gradient(point_i);
            A_matrix.template block<D, P>(D * i, D * n) = A_matrix.template block<P, D>(D * n, D * i).transpose();
        }
        A_factorized.compute(A_matrix);         // factorize
    }
    void solve() {
        const int P = PolynomialBasisGen::DimOut;
        const int D = 1 + DimIn;
        const size_t n = constraints.size();
        const int m = D * n + P;
        Eigen::Matrix<double, -1, DimOut> b;
        b.setZero(m, DimOut);
        // constraint part
        for (size_t i = 0; i < n; ++i) {
            const Value& value_i = std::get<1>(constraints[i]);
            const Gradient& gradient_i = std::get<2>(constraints[i]);
            b.template block<D, DimOut>(D * i, 0).transpose() << value_i, gradient_i;
        }
        // polynomial part is just 0
        weights = A_factorized.solve(b);        // solve
    }
    void factorize_and_solve() {
        factorize();
        solve();
    }
    Value operator()(const Point& point) const {
        const int P = PolynomialBasisGen::DimOut;
        const int D = 1 + DimIn;
        Value result = Value::Zero();
        for (size_t i = 0; i < constraints.size(); ++i) {
            const Point& point_i = std::get<0>(constraints[i]);
            // first (value) rbf part
            result += kernel(point, point_i) * weights.row(D * i).transpose();
            // second (gradient) rbf part
            result += (kernel.gradient(point, point_i) * weights.template block<DimIn, DimOut>(D * i + 1, 0)).transpose();
        }
        // polynomial part
        auto basis = PolynomialBasisGen::basis(point);
        result += (basis.transpose() * weights.bottomRows(P)).transpose();
        return result;
    }
    Gradient gradient(const Point& point) const {
        const int P = PolynomialBasisGen::DimOut;
        Gradient result = Gradient::Zero();
        // rbf part
        for (size_t i = 0; i < constraints.size(); ++i) {
            const Point& point_i = std::get<0>(constraints[i]);
            const int D = 1 + DimIn;
            // first (value) rbf part
            result += weights.row(D * i).transpose() * kernel.gradient(point, point_i);
            // second (gradient) rbf part
            result += weights.template block<DimIn, DimOut>(D * i + 1, 0).transpose() * kernel.hessian(point, point_i);
        }
        // polynomial part
        auto b_gradient = PolynomialBasisGen::gradient(point);
        result += weights.bottomRows(P).transpose() * b_gradient;
        return result;
    }
};

}

