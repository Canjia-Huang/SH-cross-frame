#pragma once
#include "range_util.hh"

namespace kt84 {
    namespace internal {
        template<typename Container, typename Iterator, typename Element>
        struct AdjacentPairIter {
            AdjacentPairIter(Container& container_, Iterator pos_)
                : container(&container_), pos(pos_)
            {}
            // minimal required member functions
            typedef std::pair<Element&, Element&> DerefType;
            DerefType operator*() const {
                Iterator pos_next(pos);
                auto& first  = *pos;
                auto& second = *(++pos_next == container->end() ? container->begin() : pos_next);
                return DerefType(first, second);
            }
            AdjacentPairIter& operator++() {
                ++pos;
                return *this;
            }
            bool operator!=(const AdjacentPairIter& rhs) const { return container != rhs.container || pos != rhs.pos; }
        private:
            Container* container;
            Iterator pos;
        };
        template<typename Container, typename Iterator>
        inline auto make_AdjacentPairIter(Container& container, Iterator pos) -> AdjacentPairIter<Container, Iterator, decltype(*pos)> {
            return AdjacentPairIter<Container, Iterator, decltype(*pos)>(container, pos);
        }
        
    }
    template<class Container>
    inline auto adjacent_pairs(Container& container, bool is_loop) -> range_util::internal::Range<internal::AdjacentPairIter<Container, decltype(container.begin()), decltype(*container.begin())>> {
        auto pos_begin = internal::make_AdjacentPairIter(container, container.begin());
        auto pos_end   = internal::make_AdjacentPairIter(container, is_loop ? container.end() : --container.end());
        return range_util::make_range(pos_begin, pos_end);
    }
}
