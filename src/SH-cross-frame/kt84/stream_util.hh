#pragma once
#include <iostream>
#include <sstream>
#include "string_util.hh"

namespace kt84 {
    namespace stream_util {
        template <typename Value, typename IStream>
        Value query_istream(IStream& stream) {
            Value value;
            stream >> value;
            return value;
        }
        template <typename IStream>
        std::string getline(IStream& stream) {
            std::string line;
            std::getline(stream, line);
            return string_util::remove_newline(line);
        }
        template <typename IStream>
        std::istringstream getline_iss(IStream& stream) {
            std::string line;
            std::getline(stream, line);
            return std::istringstream(string_util::remove_newline(line));
        }
        template <typename StringStream>
        void reset_ss(StringStream& stream, const std::string& s) {
            stream.clear();
            stream.str(s);
        }
    }
}
