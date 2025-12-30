#pragma once
#include <cmath>
#include <algorithm>
#include <vector>
#include <limits>
#include <Eigen/Geometry>
#include <boost/optional.hpp>
#include "BaryCoordT.hh"
#include "util.hh"
#include "container_util.hh"
#include "range_util.hh"

namespace kt84 {
    namespace eigen_util {
        template <typename Point>
        inline Point uniform_sample_in_triangle(const Point& p0, const Point& p1, const Point& p2, double uniform_s, double uniform_t) {
            // Generating Random Points in Triangles [Turk, Graphics Gems 1990]
            double sqrt_t = std::sqrt(uniform_t);
            double a = 1 - sqrt_t;
            double b = (1 - uniform_s) * sqrt_t;
            double c = uniform_s * sqrt_t;
            return a * p0 + b * p1 + c * p2;
        }
        template <typename Point>
        inline Point uniform_sample_in_tetrahedron(const Point& p0, const Point& p1, const Point& p2, const Point& p3, double uniform_s, double uniform_t, double uniform_u) {
            // Generating Random Points in a Tetrahedron [Rocchini and Cignoni, JGT 2001]
            if (uniform_s + uniform_t > 1.0) { // cut'n fold the cube into a prism
                uniform_s = 1.0 - uniform_s;
                uniform_t = 1.0 - uniform_t;
            }
            if (uniform_t + uniform_u > 1.0) { // cut'n fold the prism into a tetrahedron
                double tmp = uniform_u;
                uniform_u = 1.0 - uniform_s - uniform_t;
                uniform_t = 1.0 - tmp;
            } else if (uniform_s + uniform_t + uniform_u > 1.0) {
                double tmp = uniform_u;
                uniform_u = uniform_s + uniform_t + uniform_u - 1.0;
                uniform_s = 1 - uniform_t - tmp;
            }
            double a = 1 - uniform_s - uniform_t - uniform_u; // a,s,t,u are the barycentric coordinates of the random point.
            return p0 * a + p1 * uniform_s + p2 * uniform_t + p3 * uniform_u;
        }
        template <typename TMatrix>
        inline void erase_rows(TMatrix& m, std::vector<int>& row_indices) {
            if (row_indices.empty()) return;
            container_util::remove_duplicate(row_indices);
            TMatrix tmp(m);
            int cols = m.cols();
            m.resize(0, cols);
            for (auto i = row_indices.begin(); i != row_indices.end(); ++i) {
                if (i == row_indices.begin() && *i > 0) {
                    int n = *i;
                    m.resize(n, cols);
                    m = tmp.topRows(n);
                } else if (i + 1 == row_indices.end() && *i < tmp.rows() - 1) {
                    int n = tmp.rows() - 1 - *i;
                    m.conservativeResize(m.rows() + n, cols);
                    m.bottomRows(n) = tmp.bottomRows(n);
                } else if (i + 1 != row_indices.end()) {
                    int n = *(i + 1) - 1 - *i;
                    if (n > 0) {
                        m.conservativeResize(m.rows() + n, cols);
                        m.bottomRows(n) = tmp.block(*i + 1, 0, n, cols);
                    }
                }
            }
        }
        template <typename TMatrix>
        inline void erase_cols(TMatrix& m, std::vector<int>& col_indices) {
            if (col_indices.empty()) return;
            container_util::remove_duplicate(col_indices);
            TMatrix tmp(m);
            int rows = m.rows();
            m.resize(rows, 0);
            for (auto i = col_indices.begin(); i != col_indices.end(); ++i) {
                if (i == col_indices.begin() && *i > 0) {
                    int n = *i;
                    m.resize(rows, n);
                    m = tmp.leftCols(n);
                } else if (i + 1 == col_indices.end() && *i < tmp.cols() - 1) {
                    int n = tmp.cols() - 1 - *i;
                    m.conservativeResize(rows, m.cols() + n);
                    m.rightCols(n) = tmp.rightCols(n);
                } else if (i + 1 != col_indices.end()) {
                    int n = *(i + 1) - 1 - *i;
                    if (n > 0) {
                        m.conservativeResize(rows, m.cols() + n);
                        m.rightCols(n) = tmp.block(0, *i + 1, rows, n);
                    }
                }
            }
        }
        template <typename TMatrix>
        inline auto range_elements(TMatrix& m) -> decltype(range_util::make_range(m.data(), m.data() + m.size())) {
            return range_util::make_range(m.data(), m.data() + m.size());
        }
        template <typename TVector, typename TScalar>
        inline void push_back(TVector& v, TScalar s) {
            int n = v.size();
            v.conservativeResize(n + 1);
            v[n] = s;
        }
        template <typename MatrixTop, typename MatrixBottom>
        inline void append_rows(MatrixTop& m_top, MatrixBottom&& m_bottom) {
            m_top.conservativeResize(m_top.rows() + m_bottom.rows(), m_top.cols());
            m_top.bottomRows(m_bottom.rows()) << m_bottom;
        }
        template <typename MatrixLeft, typename MatrixRight>
        inline void append_cols(MatrixLeft& m_left, MatrixRight&& m_right) {
            m_left.conservativeResize(m_left.rows(), m_left.cols() + m_right.cols());
            m_left.rightCols(m_right.cols()) << m_right;
        }
        template <typename TVector>
        inline void erase_at(TVector& v, int index) {
            int n = v.size();
            auto tmp(v);
            v.resize(n - 1);
            v << tmp.head(index), tmp.tail(n - 1 - index);
        }
        template <typename TVector>
        inline void swap_xy(TVector& v) {
            std::swap(v.x(), v.y());
        }
        template <typename TAlignedBox>
        inline void bbox_add_margin(TAlignedBox& bbox, double ratio, bool is_margin_square = false) {
            if (is_margin_square) {
                typedef typename TAlignedBox::VectorType VectorType;
                VectorType margin = VectorType::Ones();
                margin *= bbox.diagonal().norm() * ratio;
                bbox.max() += margin;
                bbox.min() -= margin;
            } else {
                bbox.extend((1 + ratio) * bbox.max() - ratio * bbox.min());
                bbox.extend((1 + ratio) * bbox.min() - ratio * bbox.max());
            }
        }
        inline Eigen::Vector2d bbox_bilinear(const Eigen::AlignedBox2d& bbox, double tx, double ty) {            // (0, 0) corresponds to bbox.min, (1, 1) corresponds to bbox.max
            double x = (1 - tx) * bbox.min().x() + tx * bbox.max().x();
            double y = (1 - ty) * bbox.min().y() + ty * bbox.max().x();
            return Eigen::Vector2d(x, y);
        }
        inline Eigen::Vector3d bbox_trilinear(const Eigen::AlignedBox3d& bbox, double tx, double ty, double tz) {            // (0, 0, 0) corresponds to bbox.min, (1, 1, 1) corresponds to bbox.max
            double x = (1 - tx) * bbox.min().x() + tx * bbox.max().x();
            double y = (1 - ty) * bbox.min().y() + ty * bbox.max().x();
            double z = (1 - tz) * bbox.min().z() + tz * bbox.max().z();
            return Eigen::Vector3d(x, y, z);
        }
        inline Eigen::Vector3d orientation_color(Eigen::Vector3d d) {
            auto cx = d.x() > 0 ? Eigen::Vector3d(1, 0, 0) : Eigen::Vector3d(0, 1, 1);
            auto cy = d.y() > 0 ? Eigen::Vector3d(0, 1, 0) : Eigen::Vector3d(1, 0, 1);
            auto cz = d.z() > 0 ? Eigen::Vector3d(0, 0, 1) : Eigen::Vector3d(1, 1, 0);
            d /= std::abs<double>(d.sum());
            d = d.cwiseAbs();
            return d.x() * cx + d.y() * cy + d.z() * cz;
        }
        inline Eigen::Vector3d heat_color(double t) {
            // t     | 0    | 0.25 | 0.5   | 0.75   | 1   |
            // color | blue | cyan | green | yellow | red |
            t = util::clamp(t, 0., 1.);
            Eigen::Vector3d colors[5] = {
                Eigen::Vector3d(0, 0, 1),
                Eigen::Vector3d(0, 1, 1),
                Eigen::Vector3d(0, 1, 0),
                Eigen::Vector3d(1, 1, 0),
                Eigen::Vector3d(1, 0, 0)
            };
            int i = t < 0.25 ? 0 : t < 0.5 ? 1 : t < 0.75 ? 2 : 3;
            double s = (t - i * 0.25) * 4;
            return (1 - s) * colors[i] + s * colors[i + 1];
        }
        template <int N>
        inline int closest_axis(const Eigen::Matrix<double, N, 1, 0, N, 1>& v) {
            int result = -1;
            double v_abs_max = 0;
            for (int i = 0; i < N; ++i) {
                double v_abs = std::abs(v[i]);
                if (v_abs_max < v_abs) {
                    v_abs_max = v_abs;
                    result = i;
                }
            }
            return result;
        }
        inline Eigen::Vector2d compute_gradient(const Eigen::Vector2d& x0, const Eigen::Vector2d& x1, const Eigen::Vector2d& x2, double y0, double y1, double y2) {
            /*
                a.x0 + b = y0
                a.x1 + b = y1
                a.x2 + b = y2
                -->
                a.(x1 - x0) = y1 - y0
                a.(x2 - x0) = y2 - y0
                -->
                |(x1 - x0)^T| * a = |y1 - y0|
                |(x2 - x0)^T|       |y2 - y0|
            */
            Eigen::Matrix2d A;
            A << Eigen::RowVector2d(x1 - x0),
                 Eigen::RowVector2d(x2 - x0);
            return A.inverse() * Eigen::Vector2d(y1 - y0, y2 - y0);
        }
        inline Eigen::Vector3d compute_gradient(const Eigen::Vector3d& x0, const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, const Eigen::Vector3d& x3, double y0, double y1, double y2, double y3) {
            /*
                a.x0 + b = y0
                a.x1 + b = y1
                a.x2 + b = y2
                a.x3 + b = y3
                -->
                a.(x1 - x0) = y1 - y0
                a.(x2 - x0) = y2 - y0
                a.(x3 - x0) = y3 - y0
                -->
                |(x1 - x0)^T|       |y1 - y0|
                |(x2 - x0)^T| * a = |y2 - y0|
                |(x3 - x0)^T|       |y3 - y0|
            */
            Eigen::Matrix3d A;
            A << Eigen::RowVector3d(x1 - x0),
                 Eigen::RowVector3d(x2 - x0),
                 Eigen::RowVector3d(x3 - x0);
            return A.inverse() * Eigen::Vector3d(y1 - y0, y2 - y0, y3 - y0);
        }
        inline Eigen::Vector3d compute_gradient(const Eigen::Vector3d& x0, const Eigen::Vector3d& x1, const Eigen::Vector3d& x2, double y0, double y1, double y2) {
            /*
                Compute gradient restricted to the tangent vectors on the triangle x0-x1-x2.
                a.x0 + b = y0
                a.x1 + b = y1
                a.x2 + b = y2
                a.n      = 0                (n: normal)
                -->
                a.(x1 - x0) = y1 - y0
                a.(x2 - x0) = y2 - y0
                a.n         = 0
                -->
                |(x1 - x0)^T|       |y1 - y0|
                |(x2 - x0)^T| * a = |y2 - y0|
                | n       ^T|       |0      |
            */
            Eigen::Matrix3d A;
            A << Eigen::RowVector3d(x1 - x0),
                 Eigen::RowVector3d(x2 - x0),
                 Eigen::RowVector3d((x1 - x0).cross(x2 - x0));
            return A.inverse() * Eigen::Vector3d(y1 - y0, y2 - y0, 0);
        }
        inline Eigen::Vector2d rotate90(const Eigen::Vector2d& xy) { return Eigen::Vector2d(-xy[1], xy[0]); }
        inline double angle(const Eigen::Vector2d& d0, const Eigen::Vector2d& d1) { return std::atan2(rotate90(d0).dot(d1), d0.dot(d1)); }
        inline double angle(const Eigen::Vector3d& d0, const Eigen::Vector3d& d1) { return std::acos(d0.normalized().dot(d1.normalized())); }
        template <class T>
        inline void orthonormalize(const T& unit, T& p) {
            p -= unit.dot(p) * unit;
            p.normalize();
        }
        template <class T>
        inline T orthonormalized(const T& unit, const T& p) {
            T temp(p);
            orthonormalize(unit, temp);
            return temp;
        }
        template <class T>
        inline bool project_to_line(const T& line_v0, const T& line_v1, const T& point, Eigen::Vector2d& t) {
            /*
                x0 := line_v0
                x1 := line_v1
                y := point
                compute t (which sums up to one) such that
                    | t[0] * x0 + t[1] * x1 - y |^2
                is minimized.
                ---------------------
                u := t[1]
                (1 - u) * x0 + u * x1 =~ y
                u =~ (y - x0).dot(x1 - x0) / (x1 - x0).squaredNorm()
            */
            
            double r = (line_v1 - line_v0).squaredNorm();
            if (r == 0)
                // degenerate
                return false;
            
            t[1] = (point - line_v0).dot(line_v1 - line_v0) / r;
            t[0] = 1 - t[1];
            
            return true;
        }
        
        template <class T>
        inline bool project_to_triangle(const T& triangle_v0, const T& triangle_v1, const T& triangle_v2, const T& point, Eigen::Vector3d& t) {
            /*
                x0 := triangle_v0
                x1 := triangle_v1
                x2 := triangle_v2
                y := point
                compute t (which sums up to one) such that
                    | t[0] * x0 + t[1] * x1 + t[2] * x2 - y |^2
                is minimized.
                ---------------------
                u := t[1]
                v := t[2]
                (1 - u - v) * x0 + u * x1 + v * x2 =~ y
                (x1-x0, x2-x0) * |u| =~ y-x0
                                 |v|
                |u| =~ |(x1-x0).squaredNorm(), (x1-x0).dot(x2-x0)   |^-1 * | (x1-x0).dot(y-x0) |
                |v|    |(x1-x0).dot(x2-x0)   , (x2-x0).squaredNorm()|      | (x2-x0).dot(y-x0) |
            */
            
            T d01 = triangle_v1 - triangle_v0;
            T d02 = triangle_v2 - triangle_v0;
            T d0p = point       - triangle_v0;
            Eigen::Matrix2d M;
            M <<
                d01.squaredNorm(), d01.dot(d02),
                d01.dot(d02)     , d02.squaredNorm();
            if (M.determinant() == 0)
                // degenerate
                return false;
            
            Eigen::Vector2d b;
            b <<
                d01.dot(d0p),
                d02.dot(d0p);
            
            Eigen::Vector2d uv = ((Eigen::Matrix2d)M.inverse()) * b;
            
            t[0] = 1 - uv[0] - uv[1];
            t[1] = uv[0];
            t[2] = uv[1];
            
            return true;
        }
        template <class T>
        inline bool project_to_tetrahedron(const T& v0, const T& v1, const T& v2, const T& v3, const T& p, Eigen::Vector4d& t) {
            /*
                compute t (which sums up to one) such that
                    | t[0] * v0 + t[1] * v1 + t[2] * v2 + t[3] * v3 - p |^2
                is minimized.
                ---------------------
                u := t[1]
                v := t[2]
                w := t[3]
                (1 - u - v - w) * v0 + u * v1 + v * v2 + w * v3 =~ p
                                        |u|
                (v1-v0, v2-v0, v3-v0) * |v| =~ p-v0
                                        |w|
                |u| =~ |(v1-v0).squaredNorm(), (v1-v0).dot(v2-v0)   , (v1-v0).dot(v3-v0)   |^-1 * | (v1-v0).dot(p-v0) |
                |v|    |(v1-v0).dot(v2-v0)   , (v2-v0).squaredNorm(), (v2-v0).dot(v3-v0)   |      | (v2-v0).dot(p-v0) |
                |w|    |(v1-v0).dot(v3-v0)   , (v2-v0).dot(v3-v0)   , (v3-v0).squaredNorm()|      | (v3-v0).dot(p-v0) |
            */
            
            T d1 = v1 - v0;
            T d2 = v2 - v0;
            T d3 = v3 - v0;
            T dp = p - v0;
            Eigen::Matrix3d M;
            M <<
                d1.squaredNorm(), d1.dot(d2)      , d1.dot(d3)      ,
                d1.dot(d2)      , d2.squaredNorm(), d2.dot(d3)      ,
                d1.dot(d3)      , d2.dot(d3)      , d3.squaredNorm();
            if (M.determinant() == 0)
                // degenerate
                return false;
            
            Eigen::Vector3d b;
            b <<
                d1.dot(dp),
                d2.dot(dp),
                d3.dot(dp);
            
            Eigen::Vector3d uvw = ((Eigen::Matrix3d)M.inverse()) * b;
            
            t[0] = 1 - uvw[0] - uvw[1] - uvw[2];
            t[1] = uvw[0];
            t[2] = uvw[1];
            t[3] = uvw[2];
            
            return true;
        }
        
        template <class T>
        inline boost::optional<double> distance_to_line(const T& line_v0, const T& line_v1, const T& point, bool do_clamp = false) {
            Eigen::Vector2d t;
            if (!project_to_line(line_v0, line_v1, point, t))
                // degenrate case
                return boost::none;
            
            if (do_clamp) {
                t[0] = util::clamp(t[0], 0.0, 1.0);
                t[1] = 1 - t[0];
            }
            
            return (t[0] * line_v0 + t[1] * line_v1 - point).norm();
        }
        
        template <class T>
        inline boost::optional<double> distance_to_triangle(const T& triangle_v0, const T& triangle_v1, const T& triangle_v2, const T& point, bool do_clamp = false) {
            Eigen::Vector3d t;
            if (!project_to_triangle(triangle_v0, triangle_v1, triangle_v2, point, t))
                // degenrate case
                return boost::none;
            
            if (do_clamp && t[0] < 0 || t[1] < 0 || t[2] < 0) {
                double d0 = *distance_to_line(triangle_v0, triangle_v1, point, true);
                double d1 = *distance_to_line(triangle_v1, triangle_v2, point, true);
                double d2 = *distance_to_line(triangle_v2, triangle_v0, point, true);
                return util::min(d0, d1, d2);
            }
            
            return (t[0] * triangle_v0 + t[1] * triangle_v1 + t[2] * triangle_v2 - point).norm();
        }
        inline double triangle_area(const Eigen::Vector2d& v0, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
            Eigen::Vector2d d1 = v1 - v0;
            Eigen::Vector2d d2 = v2 - v0;
            return d1.x() * d2.y() - d1.y() * d2.x();
        }
        inline double triangle_area(const Eigen::Vector3d& v0, const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
            Eigen::Vector3d d1 = v1 - v0;
            Eigen::Vector3d d2 = v2 - v0;
            return d1.cross(d2).norm() / 2;
        }
        template <class T>
        inline bool intersection(const T& line0_p0, const T& line0_p1, const T& line1_p0, const T& line1_p1, double& line0_coordinate, double& line1_coordinate) {
            /*
                notation:
                    v0 := line0_p0
                    v1 := line0_p1
                    w0 := line1_p0
                    w1 := line1_p1
                    s  := line0_coordinate
                    t  := line1_coordinate
                seek for (s, t) which minimizes:
                    |(1 - s) * v0 + s * v1 - (1 - t) * w0 - t * w1|^2
                least square sense:
                    | v1-v0, -w1+w0 | * |s| =~ -v0+w0
                                        |t|
            */
            T d0 = line0_p1 - line0_p0;
            T d1 = line1_p1 - line1_p0;
            T e  = line1_p0 - line0_p0;
            double d0d0 = d0.squaredNorm();
            double d0d1 = d0.dot(d1);
            double d1d1 = d1.squaredNorm();
            Eigen::Matrix2d A;
            A <<
                d0d0, -d0d1,
                -d0d1, d1d1;
            
            if (A.determinant() == 0)
                // two lines are parallel
                return false;
            
            Eigen::Vector2d b(d0.dot(e), -d1.dot(e));
            Eigen::Vector2d st = ((Eigen::Matrix2d)A.inverse()) * b;
            
            line0_coordinate = st[0];
            line1_coordinate = st[1];
            return true;
        }
        template <typename T>
        inline std::vector<T> eigen_vectorx_to_std_vector(const Eigen::Matrix<T, -1, 1>& eigen_vectorx) {
            int n = eigen_vectorx.rows();
            std::vector<T> result(n);
            for (int i = 0; i < n; ++i)
                result[i] = eigen_vectorx[i];
            return result;
        }
        template <typename T>
        inline Eigen::Matrix<T, -1, 1> std_vector_to_eigen_vectorx(const std::vector<T>& std_vector) {
            int n = std_vector.size();
            Eigen::Matrix<T, -1, 1> result = Eigen::Matrix<T, -1, 1>::Zero(n);
            for (int i = 0; i < n; ++i)
                result[i] = std_vector[i];
            return result;
        }
        template <typename TVector>
        inline int max_axis(const TVector& v) {
            int index_max = -1;
            typename TVector::Scalar value_max = -std::numeric_limits<typename TVector::Scalar>::max();
            for (int i = 0; i < v.size(); ++i) {
                if (value_max < v[i]) {
                    value_max = v[i];
                    index_max = i;
                }
            }
            return index_max;
        }
        template <typename TVector>
        inline int min_axis(const TVector& v) {
            int index_min = -1;
            typename TVector::Scalar value_min = std::numeric_limits<typename TVector::Scalar>::max();
            for (int i = 0; i < v.size(); ++i) {
                if (value_min < v[i]) {
                    value_min = v[i];
                    index_min = i;
                }
            }
            return index_min;
        }
        template <typename TVector>
        inline void rotate(TVector& v) {
            TVector tmp(v);
            int n = v.size();
            for (int i = 0; i < n; ++i)
                v[i] = tmp[(i + 1) % n];
        }
        template <typename TVector>
        inline void reverse(TVector& v) {
            TVector tmp(v);
            int n = v.size();
            for (int i = 0; i < n; ++i)
                v[i] = tmp[n - 1 - i];
        }
        template <typename Scalar>
        inline Scalar mat1x1_to_scalar(const Eigen::Matrix<Scalar, 1, 1>& m) {
            return m(0, 0);
        }
        template <typename Scalar>
        inline Eigen::Matrix<Scalar, 1, 1> scalar_to_mat1x1(Scalar val) {
            Eigen::Matrix<Scalar, 1, 1> m;
            m(0, 0) = val;
            return m;
        }
    }
}
