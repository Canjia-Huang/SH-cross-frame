#pragma once
#include <sstream>
#include <ios>
#include <numeric>
#include <functional>

namespace kt84 {
    namespace string_util {
        inline int hamming_distance(const std::string& str1, const std::string str2) {
            if (str1.size() != str2.size())
                return -1;
            return std::inner_product(str1.begin(), str1.end(), str2.begin(), 0, std::plus<int>(), std::not_equal_to<char>());
        }
        inline std::string remove_newline(const std::string& s) {
            std::string result = s;
            for (auto i = result.begin(); i != result.end(); ) {
                auto c = *i;
                (c == '\n' || c == '\r') ? result.erase(i) : i++;
            }
            return result;
        }
        namespace internal {
            struct Cat {
                std::string str;
                Cat() = default;
                // generic ctor
                template <typename Arg>
                Cat(const Arg& arg) {
                    *this += arg;
                }
                template <typename Arg>
                Cat& operator+=(Arg&& arg) {
                    std::ostringstream oss;
                    oss << str << std::boolalpha << arg; 
                    str = oss.str();
                    return *this;
                }
                template <typename Arg>
                Cat operator+(Arg&& arg) {
                    return Cat(*this) += arg;
                }
                operator std::string() const { return str; }
                const char* c_str() const { return str.c_str(); }
            };
        }
        inline internal::Cat cat() { return {}; }
        template <typename Arg>
        inline internal::Cat cat(Arg&& arg) { return {arg}; }
    }
}
