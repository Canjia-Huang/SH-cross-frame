//
// Created by huangcanjia on 25-11-27.
//

#include <gtest/gtest.h>
#include "utils/log.h"

namespace
{
    TEST(DepsTest, spdlog)
    {
        LOG::TRACE("Hello World!");
        LOG::DEBUG("Hello World!");
        LOG::INFO("Hello World!");
        LOG::WARN("Hello World!");
        LOG::ERROR("Hello World!");
        LOG::CRITICAL("Hello World!");

        SUCCEED();
    }
}