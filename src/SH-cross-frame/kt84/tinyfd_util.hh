#pragma once
#include <array>
#include <vector>
#include <string>
#include <tinyfiledialogs.h>

namespace kt84 {
    namespace tinyfd_util {
        inline std::string inputBox(
            char const * const aTitle, /* "" */
            char const * const aMessage, /* "" may NOT contain \n nor \t (on windows)*/
            char const * const aDefaultInput)  /* "" , if NULL it's a passwordBox */
        {
            std::string result;
            auto c_str = tinyfd_inputBox(aTitle, aMessage, aDefaultInput);
            if (c_str)
                result = c_str;
            return result;
        }
        inline std::string saveFileDialog(
            char const * const aTitle, /* "" */
            char const * const aDefaultPathAndFile, /* "" */
            const std::vector<const char*>& aFileFilters) /* {"*.jpg", "*.png"} */
        {
            std::string result;
            auto c_str = tinyfd_saveFileDialog(aTitle, aDefaultPathAndFile, aFileFilters.size(), &aFileFilters[0]);
            if (c_str)
                result = c_str;
            return result;
        }
        inline std::string openFileDialog(
            char const * const aTitle, /* "" */
            char const * const aDefaultPathAndFile, /* "" */
            const std::vector<const char*>& aFileFilters, /* {"*.jpg","*.png"} */
            bool aAllowMultipleSelects) /* 0 or 1 */
            /* in case of multiple files, the separator is | */
        {
            std::string result;
            auto c_str = tinyfd_openFileDialog(aTitle, aDefaultPathAndFile, aFileFilters.size(), &aFileFilters[0], aAllowMultipleSelects);
            if (c_str)
                result = c_str;
            return result;
        }
        inline std::string selectFolderDialog(
            char const * const aTitle, /* "" */
            char const * const aDefaultPath) /* "" */
        {
            std::string result;
            auto c_str = tinyfd_selectFolderDialog(aTitle, aDefaultPath);
            if (c_str)
                result = c_str;
            return result;
        }
        inline std::string colorChooser(
            char const * const aTitle, /* "" */
            char const * const aDefaultHexRGB, /* NULL or "#FF0000" */
            const std::array<unsigned char, 3>& aDefaultRGB, /* { 0 , 255 , 255 } */
            std::array<unsigned char, 3>& aoResultRGB) /* { 0 , 0 , 0 } */
	        /* returns the hexcolor as a string "#FF0000" */
	        /* aoResultRGB also contains the result */
	        /* aDefaultRGB is used only if aDefaultHexRGB is NULL */
	        /* aDefaultRGB and aoResultRGB can be the same array */
        {
            std::string result;
            auto c_str = tinyfd_colorChooser(aTitle, aDefaultHexRGB, aDefaultRGB.data(), aoResultRGB.data());
            if (c_str)
                result = c_str;
            return result;
        }
    }
}
