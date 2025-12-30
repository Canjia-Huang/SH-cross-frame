#pragma once
#include <iterator>
#include <boost/range/algorithm.hpp>
#include <boost/optional.hpp>

namespace kt84 {
namespace container_util {
    template <class Container>
    inline bool insert_if_nonexistent(Container& c, const decltype(*c.begin())& val) {
        if (boost::range::find(c, val) == c.end()) {
            c.insert(val);
            return true;
        }
        return false;
    }
    template<class Map>
    inline boost::optional<typename Map::mapped_type> at_optional(Map& map, const typename Map::key_type& key) {
        auto p = map.find(key);
        if (p == map.end())
            return boost::none;
        else
            return p->second;
    }
    template<class Container>
    inline int mod_index(const Container& c, int index) {
        while (index < 0) index += c.size();
        return index % c.size();
    }
    template<class Container>
    inline auto at_mod(Container& c, int index) -> decltype(c[index]) {
        return c[mod_index(c,index)];
    }
    template<class Container>
    inline auto at_mod(const Container& c, int index) -> decltype(c[index]) {
        return c[mod_index(c,index)];
    }
    /*
    Boost equivalent `boost::range::remove_erase_if' exists in <boost/range/algorithm_ext/erase.hpp>
    template<class Container, class Predicate>
    inline void remove_if(Container& c, Predicate pred) {
        c.erase(boost::range::remove_if(c, pred), c.end());
    }
    */
    template<class Container> 
    inline void bring_front(Container& c, const decltype(*c.begin())& val) {
        boost::range::rotate(c, boost::range::find(c, val));
    }
    template<class Container> 
    inline auto erase_at(Container& c, int index) -> decltype(c.erase(c.begin())) {
        auto pos = c.begin();
        std::advance(pos, index);
        return c.erase(pos);
    }
    template<class Container>
    inline void remove_duplicate(Container& c) {
        boost::range::sort(c);
        auto new_last = boost::range::unique(c).end();
        c.erase(new_last, c.end());
    }
    template <class Container>
    inline auto max(const Container& c, int& index) -> decltype(*c.begin()) {
        auto i = c.begin();
        auto result = *i;
        index = 0;
        for (++i; i != c.end(); ++i, ++index)
            if (result < *i) result = *i;
        return result;
    }
    template <class Container>
    inline auto min(const Container& c, int& index) -> decltype(*c.begin()) {
        auto i = c.begin();
        auto result = *i;
        index = 0;
        for (++i; i != c.end(); ++i, ++index)
            if (result > *i) result = *i;
        return result;
    }
    template <class Container>
    inline auto max(const Container& c) -> decltype(*c.begin()) {
        int index;
        return max(c, index);
    }
    template <class Container>
    inline auto min(const Container& c) -> decltype(*c.begin()) {
        int index;
        return min(c, index);
    }
    template <class Container>
    inline auto sum(const Container& c) -> decltype(*c.begin()) {
        auto i = c.begin();
        auto result = *i;
        for (++i; i != c.end(); ++i)
            result += *i;
        return result;
    }
    template <class Container, class Range>
    inline void append(Container& c, const Range& range) {
        boost::range::copy(range, std::back_inserter(c));
    }
    namespace internal {
        template <class Container>
        struct All {
            const Container& container;
            All(const Container& container): container(container) {}
            template <typename Rhs> bool operator==(const Rhs& rhs) const { for (auto& e : container) if (!(e == rhs)) return false; return true; }
            template <typename Rhs> bool operator!=(const Rhs& rhs) const { for (auto& e : container) if (!(e != rhs)) return false; return true; }
            template <typename Rhs> bool operator< (const Rhs& rhs) const { for (auto& e : container) if (!(e <  rhs)) return false; return true; }
            template <typename Rhs> bool operator> (const Rhs& rhs) const { for (auto& e : container) if (!(e >  rhs)) return false; return true; }
            template <typename Rhs> bool operator<=(const Rhs& rhs) const { for (auto& e : container) if (!(e <= rhs)) return false; return true; }
            template <typename Rhs> bool operator>=(const Rhs& rhs) const { for (auto& e : container) if (!(e >= rhs)) return false; return true; }
        };
        template <class Container>
        struct Any {
            const Container& container;
            Any(const Container& container): container(container) {}
            template <typename Rhs> bool operator==(const Rhs& rhs) const { for (auto& e : container) if (e == rhs) return true; return false; }
            template <typename Rhs> bool operator!=(const Rhs& rhs) const { for (auto& e : container) if (e != rhs) return true; return false; }
            template <typename Rhs> bool operator< (const Rhs& rhs) const { for (auto& e : container) if (e <  rhs) return true; return false; }
            template <typename Rhs> bool operator> (const Rhs& rhs) const { for (auto& e : container) if (e >  rhs) return true; return false; }
            template <typename Rhs> bool operator<=(const Rhs& rhs) const { for (auto& e : container) if (e <= rhs) return true; return false; }
            template <typename Rhs> bool operator>=(const Rhs& rhs) const { for (auto& e : container) if (e >= rhs) return true; return false; }
        };
    }
    template <class Container> inline internal::All<Container> all(const Container& c) { return internal::All<Container>(c); }
    template <class Container> inline internal::Any<Container> any(const Container& c) { return internal::Any<Container>(c); }
}
}
