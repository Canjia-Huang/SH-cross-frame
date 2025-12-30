//
// Created by Canjia Huang <canjia7@gmail.com> on 2025/12/3.
//

#ifndef POLYGEN_TEST_UTILS_H
#define POLYGEN_TEST_UTILS_H

#include <gtest/gtest.h>

namespace
{
    inline std::string get_current_test_name(){
        const testing::TestInfo* const current_test_info = testing::UnitTest::GetInstance()->current_test_info();
        return std::string("test")
                + "_"
                + std::string(current_test_info->test_case_name())
                + "_"
                + std::string(current_test_info->name());
    }
}

#endif //POLYGEN_TEST_UTILS_H