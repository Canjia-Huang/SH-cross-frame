// =============================================================================
// This file contains code derived from the following source(s):
//   Original Repository: https://kenshi84.github.io/misc/frame3d.zip
//
// Copyright (c) Kenshi Takayama
// -----------------------------------------------------------------------------
// Modifications made by Canjia Huang on 2025-12-30:
//   - Use gtest
//   - Modify some headers
//   - Remove visualization
//   - Replace `cout`/`scout` -> spdlog LOG
// =============================================================================

#include "SH-cross-frame/frame3d.hh"
#include "SH-cross-frame/kt84/util.hh"
#include "utils/log.h"
#include <gtest/gtest.h>

using namespace std;
using namespace kt84;
// using namespace kt84::graphics_util;

static const double pi = kt84::util::pi();

struct Globals {
    frame3d::ZYZVector zyz_0 = { 0, 0, 0 };
    frame3d::ZYZVector zyz_1 = { 0, 0, 0 };
    frame3d::ZYZVector zyz_t;
    frame3d::SHVector sh_0, sh_1, sh_t;
    double t;
    bool verbose = false;
} g;

void interpolate() {
    g.sh_0 = frame3d::zyz2sh(g.zyz_0);
    g.sh_1 = frame3d::zyz2sh(g.zyz_1);
    g.sh_t = (1 - g.t) * g.sh_0 + g.t * g.sh_1;
    g.zyz_t = frame3d::sh2zyz(g.sh_t);

    LOG::INFO("sh_0: {},{},{},{},{},{},{},{},{}", g.sh_0[0],g.sh_0[1],g.sh_0[2],g.sh_0[3],g.sh_0[4],g.sh_0[5],g.sh_0[6],g.sh_0[7],g.sh_0[8]);
    LOG::INFO("sh_1: {},{},{},{},{},{},{},{},{}", g.sh_1[0],g.sh_1[1],g.sh_1[2],g.sh_1[3],g.sh_1[4],g.sh_1[5],g.sh_1[6],g.sh_1[7],g.sh_1[8]);
    LOG::INFO("sh_t: {},{},{},{},{},{},{},{},{}", g.sh_t[0],g.sh_t[1],g.sh_t[2],g.sh_t[3],g.sh_t[4],g.sh_t[5],g.sh_t[6],g.sh_t[7],g.sh_t[8]);
    LOG::INFO("zyz_t: {},{},{}", g.zyz_t[0],g.zyz_t[1],g.zyz_t[2]);
    LOG::INFO("fitting error:{}", (g.sh_t - frame3d::zyz2sh(g.zyz_t)).norm());
}

TEST(SimpleTest, test) {
    g.zyz_0.setRandom();
    g.zyz_1.setRandom();
    g.zyz_0 += frame3d::ZYZVector::Constant(1);
    g.zyz_1 += frame3d::ZYZVector::Constant(1);
    g.zyz_0 *= pi;
    g.zyz_1 *= pi;
    g.t = 0.5;

    interpolate();
}