// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut
// =============================================================================
//
// Guard: the OptiX shader sources the runtime actually compiles must match the sources in the
// repository.
//
// Chrono::Sensor compiles its .cu shaders at runtime with NVRTC, reading them from a COPY in the
// build tree (SHADER_DIR) rather than from the source tree. The copy used to be made at configure
// time with file(COPY), so editing a shader and running a plain build left the old copy in place:
// the build succeeded, nothing warned, and the program silently ran the previous shader. That trap
// cost real debugging time and nearly produced a wrong conclusion about whether a fix worked, so
// the copy is now a build-time rule with a proper dependency.
//
// This test exists because a build rule that is correct today can be broken tomorrow by an
// unrelated CMake change, and the failure mode is silent. Rather than trying to drive a build from
// inside a test, which would mean invoking the build system recursively, it asserts the INVARIANT
// that the rule exists to maintain: every staged shader is byte-identical to its source. If staging
// is ever skipped, disabled, or wired to the wrong dependency, the staged copy drifts from the
// source and this test fails at the next ctest run.
//
// Line endings are normalised before comparison, because git may check the source out with CRLF
// while the copy is made verbatim, and a line-ending difference is not staleness.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"

// Both directories are injected by CMake so the test cannot disagree with the build about where
// the shaders live. CH_SHADER_SOURCE_DIR is the repository copy, CH_STAGED_SHADER_DIR is what the
// runtime reads.
#ifndef CH_SHADER_SOURCE_DIR
    #error "CH_SHADER_SOURCE_DIR must be defined by the build"
#endif
#ifndef CH_STAGED_SHADER_DIR
    #error "CH_STAGED_SHADER_DIR must be defined by the build"
#endif
// The staged shader file names, also injected by CMake, COMMA-separated. Comma rather than
// semicolon because a semicolon is CMake's own list separator: passing the list verbatim expands
// into one -D flag per shader and mangles the command line. Taking the list from the build rather
// than globbing the directory is deliberate: a glob of the staged directory would pass vacuously if
// staging had produced nothing at all.
#ifndef CH_STAGED_SHADER_NAMES
    #error "CH_STAGED_SHADER_NAMES must be defined by the build"
#endif

namespace {

std::vector<std::string> Split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) {
            if (!cur.empty())
                out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    if (!cur.empty())
        out.push_back(cur);
    return out;
}

// Whole file with CRLF and lone CR folded to LF, so a checkout convention cannot look like drift.
bool ReadNormalised(const std::string& path, std::string* out) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good())
        return false;
    std::string raw((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    out->clear();
    out->reserve(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
        if (raw[i] == '\r') {
            if (i + 1 < raw.size() && raw[i + 1] == '\n')
                continue;    // CRLF: drop the CR, keep the LF
            out->push_back('\n');
            continue;        // lone CR
        }
        out->push_back(raw[i]);
    }
    return true;
}

}  // namespace

// Every shader the build staged must still equal its source. A mismatch means the runtime would
// compile something other than what is in the repository.
TEST(ChOptixShaderStaging, staged_shaders_match_their_sources) {
    const std::vector<std::string> names = Split(CH_STAGED_SHADER_NAMES, ',');

    // Guard the premise. If the injected list were empty this test would pass while checking
    // nothing at all, which is the exact failure mode it is meant to prevent elsewhere.
    ASSERT_FALSE(names.empty())
        << "no staged shader names were injected by the build, so this test would verify nothing";
    std::cout << "  checking " << names.size() << " staged shader(s)" << std::endl;
    std::cout << "  source: " << CH_SHADER_SOURCE_DIR << std::endl;
    std::cout << "  staged: " << CH_STAGED_SHADER_DIR << std::endl;

    size_t compared = 0;
    for (const std::string& name : names) {
        const std::string src = std::string(CH_SHADER_SOURCE_DIR) + "/" + name;
        const std::string staged = std::string(CH_STAGED_SHADER_DIR) + "/" + name;

        std::string a, b;
        ASSERT_TRUE(ReadNormalised(src, &a)) << "cannot read shader source " << src;
        ASSERT_TRUE(ReadNormalised(staged, &b))
            << "staged shader missing: " << staged
            << "\n  the build did not stage this file, so the runtime would compile a stale copy "
               "or fail to find it";
        EXPECT_EQ(a, b) << "STALE SHADER: " << name
                        << "\n  source: " << src << " (" << a.size() << " bytes)"
                        << "\n  staged: " << staged << " (" << b.size() << " bytes)"
                        << "\n  The build-tree copy the runtime compiles differs from the source in"
                           " the repository. Staging did not run for this file, so an edit to the"
                           " shader is silently not taking effect. Check the"
                           " Chrono_sensor_shader_copy target and its dependency on Chrono_sensor"
                           " in src/chrono_sensor/CMakeLists.txt.";
        ++compared;
    }
    EXPECT_EQ(compared, names.size());
}
