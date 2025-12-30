//
// Created by huangcanjia on 25-12-30.
//

#ifndef ENVIRONMENT_GEOGRAM_H
#define ENVIRONMENT_GEOGRAM_H

#include <geogram/basic/command_line_args.h>
#include <geogram/basic/common.h>
#include <gtest/gtest.h>

class GeogramTestEnvironment final : public testing::Environment {
public:
    void SetUp() override {
        GEO::initialize(GEO::GEOGRAM_INSTALL_ALL);
        GEO::CmdLine::import_arg_group("standard");
        GEO::CmdLine::import_arg_group("algo");
    }
};


#endif //ENVIRONMENT_GEOGRAM_H
