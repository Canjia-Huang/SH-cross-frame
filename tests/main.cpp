//
// Created by huangcanjia on 25-11-27.
//

#include <gtest/gtest.h>
#include "test_environment_spdlog.h"

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);

    testing::AddGlobalTestEnvironment(new SpdlogTestEnvironment);

    return RUN_ALL_TESTS();
}