#pragma once
#include <utility>

namespace OpenVolumeMesh {
    template <class T> T begin(const std::pair<T, T>& p) { return p.first ; }
    template <class T> T end  (const std::pair<T, T>& p) { return p.second; }
}
