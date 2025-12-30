#pragma once

namespace kt84 {
    template <typename T>
    struct Average {
        T sum;
        int count = 0;
        Average() = default;
        Average(const T& zero) {
            init(zero);
        }
        void init(const T& zero) {
            sum = zero;
            count = 0;
        }
        void update(const T& value) {
            sum += value;
            ++count;
        }
        T value() const {
            return sum * (1.0 / count);
        }
    };
}
