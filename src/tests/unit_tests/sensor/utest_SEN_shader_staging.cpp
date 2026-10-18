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
// The comparison is over RAW BYTES, deliberately. An earlier version of this test folded CRLF and
// lone CR to LF first, on the theory that git might check the source out with CRLF while the copy
// was made verbatim. That reasoning was wrong, and the normalisation only weakened the guard:
// `cmake -E copy_if_different` copies verbatim, so the staged bytes always equal the source bytes
// AT THE MOMENT OF STAGING, whatever the line endings are. A CRLF checkout therefore cannot produce
// a false positive here. Conversely, if the source's bytes later differ from the staged copy in any
// way, including line endings alone, that IS the state this guard exists to catch: the build has not
// restaged since the source changed. Folding line endings would have hidden exactly that case.
//
// WHAT A CLEAN RUN OF THIS TEST DOES AND DOES NOT PROVE.
// On a from-scratch build, staging has just run, so source and staged copy always match and this
// test passes whether or not the dependency edge actually works. What a clean run DOES catch is
// staging being removed or broken outright: the staged file is then absent and the assertion below
// names it. That is why the test is worth running under ctest even though it cannot, on its own,
// prove the property the staging rule was written for.
//
// The incremental case, which is the original defect, can only be checked by hand:
//
//   1. build normally, then append a line to a shader source:
//        echo "// probe" >> <source>/src/chrono_sensor/optix/shaders/camera_raygen.cu
//   2. run an ORDINARY build of this target (add --config Release on a multi-config generator):
//        cmake --build . --target utest_SEN_shader_staging
//   3. run this test again; it must still PASS, which proves the edit was restaged:
//        ctest -R shader_staging --output-on-failure      (add -C Release if multi-config)
//   4. restore the shader:
//        git checkout -- src/chrono_sensor/optix/shaders/camera_raygen.cu
//
// If step 3 FAILS, the build is no longer restaging edited shaders and the trap described above is
// back. Measured cost of that procedure on Linux with Ninja: 0.05 s, because nothing recompiles.
// With CH_USE_SENSOR_NVRTC on, the shaders are compiled at runtime, so the only work is one copy.
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

// Whole file, exact bytes. Opened in binary mode so no platform translates anything on the way in.
bool ReadRawBytes(const std::string& path, std::string* out) {
    std::ifstream f(path, std::ios::binary);
    if (!f.good())
        return false;
    out->assign((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
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
        ASSERT_TRUE(ReadRawBytes(src, &a)) << "cannot read shader source " << src;
        ASSERT_TRUE(ReadRawBytes(staged, &b))
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
                           " in src/chrono_sensor/CMakeLists.txt."
                        << "\n  To check incremental staging by hand, see the procedure in the"
                           " header comment of this file.";
        ++compared;
    }
    EXPECT_EQ(compared, names.size());
}
