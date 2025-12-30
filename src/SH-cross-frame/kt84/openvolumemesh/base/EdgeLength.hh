#pragma once

namespace kt84 {
    namespace openvolumemesh {
        struct EdgeLength_EdgeTraits {
            double edgeLength = 0;
        };
        template <class Base>
        struct EdgeLength : public Base {
            void edgeLength_compute() {
                for (auto e : Base::edges())
                    Base::data(e).edgeLength = Base::length(e);
            }
        };
    }
}
