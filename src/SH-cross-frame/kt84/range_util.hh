#pragma once
#include <utility>
#include <boost/iterator/reverse_iterator.hpp>

namespace kt84 {
    namespace range_util {
        namespace internal {
            template <class Iterator>
            struct Range {
                Range(Iterator begin_, Iterator end_) : begin_(begin_), end_(end_) {}
                Iterator begin() const { return begin_; }
                Iterator end  () const { return end_; }
                Iterator begin_, end_;
            };
        }
        template <typename Iterator>
        inline internal::Range<Iterator> make_range(Iterator begin_, Iterator end_) {
            return internal::Range<Iterator>(begin_, end_);
        }
        template <typename Iterator>
        inline auto make_range(const std::pair<Iterator, Iterator>& p) -> decltype(make_range(p.first, p.second)) {
            return make_range(p.first, p.second);
        }
        template <class Container>
        inline auto make_reverse_range(Container& container) -> decltype(make_range(container.rbegin(), container.rend())) {
            return make_range(container.rbegin(), container.rend());
        }
        template <typename Iterator>
        inline auto make_reverse_range(Iterator begin_, Iterator end_) -> decltype(make_range(boost::make_reverse_iterator(end_), boost::make_reverse_iterator(begin_))) {
            return make_range(boost::make_reverse_iterator(end_), boost::make_reverse_iterator(begin_));
        }
    }
}
