#pragma once
#include <cmath>
#include "EdgeLength.hh"

namespace kt84 {
    namespace openvolumemesh {
        struct TetFaceArea_FaceTraits {
            double tetFaceArea = 0;
        };
        using TetFaceArea_EdgeTraits = EdgeLength_EdgeTraits;
        template <class Base>
        struct TetFaceArea : public EdgeLength<Base> {
            void tetFaceArea_compute() {
                EdgeLength<Base>::edgeLength_compute();
                for (auto f : Base::faces()) {
                    auto e = Base::face_edges(f);
                    // https://en.wikipedia.org/wiki/Heron%27s_formula
                    double a = Base::data(e[0]).edgeLength;
                    double b = Base::data(e[1]).edgeLength;
                    double c = Base::data(e[2]).edgeLength;
                    double s = (a + b + c) / 2;
                    Base::data(f).tetFaceArea = std::sqrt(s * (s - a) * (s - b) * (s - c));
                }
            }
        };
    }
}
