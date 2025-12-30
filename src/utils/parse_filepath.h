//
// Created by huangcanjia on 25-10-15.
//

#ifndef PARSE_FILEPATH_H
#define PARSE_FILEPATH_H

#include <string>
#include <filesystem>

 /**
 * Get the file name from a file path.
 * @param path
 * @return file name or empty string
 */
inline std::string get_filename(
    const std::string& path
    ) {
    const std::filesystem::path inPath(path);

    const std::string filename_w_ex = inPath.filename().string();
    if (const std::size_t pos = filename_w_ex.find_last_of('.');
        pos != std::string::npos)
        return filename_w_ex.substr(0, pos);
    return "";
}

/**
 * Get the extension from a file path
 * @param path
 * @return extension or empty string
 */
inline std::string get_extension(
    const std::string& path
    ) {
    const std::filesystem::path inPath(path);

    const std::string filename_w_ex = inPath.filename().string();
    if (const std::size_t pos = filename_w_ex.find_last_of('.');
        pos != std::string::npos)
        return filename_w_ex.substr(pos + 1);
    return "";
}

inline std::string get_absolute_file_path(
    const std::string& path
    ) {
    if (!std::filesystem::exists(path))
        return "";
    return std::filesystem::canonical(std::filesystem::absolute(path)).string();
}

/**
 * Get the parent path from a file path.
 * @param path
 * @return parent path
 */
inline std::string get_parent_path(
    const std::string& path
    ) {
    if (!std::filesystem::exists(path))
        return "";
    const std::filesystem::path absolute_path = std::filesystem::canonical(std::filesystem::absolute(path));
    return absolute_path.parent_path().string() + "/";
}

#endif //PARSE_FILEPATH_H
