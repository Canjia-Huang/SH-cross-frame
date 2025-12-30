#pragma once
#include <cmath>
#include "EdgeLength.hh"

namespace kt84 {
    namespace openvolumemesh {
        struct TetCellVolume_CellTraits {
            double tetCellVolume = 0;
        };
        using TetCellVolume_EdgeTraits = EdgeLength_EdgeTraits;
        template <class Base>
        struct TetCellVolume : public EdgeLength<Base> {
            void tetCellVolume_compute() {
                Base::edgeLength_compute();
                for (auto tet : Base::cells()) {
                    //auto e = Base::face_edges(f);
                    // Heron-like formula (http://en.wikipedia.org/wiki/Tetrahedron#Heron-type_formula_for_the_volume_of_a_tetrahedron)
                    double UVW[3], uvw[3];
                    int i = 0;
                    auto f = Base::cell_faces(tet).front();
                    for (auto e1 : Base::face_edges(f)) {
                        for (auto e2 : Base::cell_edges(tet)) {
                            if (!Base::is_adjacent(e1, e2)) {
                                UVW[i] = Base::length(e1);
                                uvw[i++] = Base::length(e2);
                                break;
                            }
                        }
                    }
                    double U = UVW[0];
                    double V = UVW[1];
                    double W = UVW[2];
                    double u = uvw[0];
                    double v = uvw[1];
                    double w = uvw[2];
                    double X = (w - U + v) * (U + v + w);
                    double Y = (u - V + w) * (V + w + u);
                    double Z = (v - W + u) * (W + u + v);
                    double x = (U - v + w) * (v - w + U);
                    double y = (V - w + u) * (w - u + V);
                    double z = (W - u + v) * (u - v + W);
                    double a = std::sqrt(x * Y * Z);
                    double b = std::sqrt(y * Z * X);
                    double c = std::sqrt(z * X * Y);
                    double d = std::sqrt(x * y * z);
                    Base::data(tet).tetCellVolume = std::sqrt((-a + b + c + d) * (a - b + c + d) * (a + b - c + d) * (a + b + c - d)) / (192 * u * v * w);
                }
            }
        };
    }
}
