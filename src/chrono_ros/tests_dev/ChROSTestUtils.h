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
// Authors: Patrick Chen
// =============================================================================
//
// Minimal assertion/reporting harness for the chrono_ros dev-time tests
// (tests_dev/README.md explains how these relate to the Chrono CI suite).
//
// =============================================================================

#ifndef CH_ROS_TEST_UTILS_H
#define CH_ROS_TEST_UTILS_H

#include <cstdint>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace chros_test {

struct Failure {
    std::string message;
};

#define CHECK(cond)                                                                                  \
    do {                                                                                             \
        if (!(cond)) {                                                                               \
            std::ostringstream oss_;                                                                 \
            oss_ << __FILE__ << ":" << __LINE__ << ": CHECK failed: " << #cond;                      \
            throw chros_test::Failure{oss_.str()};                                                   \
        }                                                                                            \
    } while (0)

#define CHECK_EQ(a, b)                                                                               \
    do {                                                                                             \
        const auto va_ = (a);                                                                        \
        const auto vb_ = (b);                                                                        \
        if (!(va_ == vb_)) {                                                                         \
            std::ostringstream oss_;                                                                 \
            oss_ << __FILE__ << ":" << __LINE__ << ": CHECK_EQ failed: " << #a << " (" << va_        \
                 << ") != " << #b << " (" << vb_ << ")";                                             \
            throw chros_test::Failure{oss_.str()};                                                   \
        }                                                                                            \
    } while (0)

/// Expect 'expr' to throw ExType whose what() contains 'substring'.
#define CHECK_THROWS(expr, ExType, substring)                                                        \
    do {                                                                                             \
        bool caught_ = false;                                                                        \
        try {                                                                                        \
            (void)(expr);                                                                            \
        } catch (const ExType& e_) {                                                                 \
            caught_ = true;                                                                          \
            if (std::string(e_.what()).find(substring) == std::string::npos) {                       \
                std::ostringstream oss_;                                                             \
                oss_ << __FILE__ << ":" << __LINE__ << ": exception message \"" << e_.what()         \
                     << "\" does not contain \"" << substring << "\"";                               \
                throw chros_test::Failure{oss_.str()};                                               \
            }                                                                                        \
        }                                                                                            \
        if (!caught_) {                                                                              \
            std::ostringstream oss_;                                                                 \
            oss_ << __FILE__ << ":" << __LINE__ << ": expected " << #ExType << " from: " << #expr;   \
            throw chros_test::Failure{oss_.str()};                                                   \
        }                                                                                            \
    } while (0)

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& Registry() {
    static std::vector<TestCase> tests;
    return tests;
}

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) { Registry().push_back({name, std::move(fn)}); }
};

#define TEST(name)                                                                                   \
    static void test_##name();                                                                       \
    static chros_test::Registrar registrar_##name(#name, test_##name);                               \
    static void test_##name()

inline int RunAll(const char* suite) {
    int failures = 0;
    for (const auto& test : Registry()) {
        try {
            test.fn();
            std::cout << "[ OK ] " << suite << "." << test.name << "\n";
        } catch (const Failure& f) {
            failures++;
            std::cout << "[FAIL] " << suite << "." << test.name << "\n       " << f.message << "\n";
        } catch (const std::exception& e) {
            failures++;
            std::cout << "[FAIL] " << suite << "." << test.name << "\n       unexpected exception: " << e.what()
                      << "\n";
        }
    }
    std::cout << (failures == 0 ? "ALL PASSED" : "FAILURES: " + std::to_string(failures)) << " ("
              << Registry().size() << " test(s))\n";
    return failures == 0 ? 0 : 1;
}

/// Compare a byte vector against expected bytes with a readable diff.
inline void CheckBytes(const std::vector<uint8_t>& actual, const std::vector<uint8_t>& expected, const char* what) {
    if (actual == expected) {
        return;
    }
    std::ostringstream oss;
    oss << what << ": byte mismatch (actual " << actual.size() << " bytes, expected " << expected.size() << ")\n";
    const size_t n = std::max(actual.size(), expected.size());
    for (size_t i = 0; i < n; i++) {
        char line[96];
        const int a = i < actual.size() ? actual[i] : -1;
        const int e = i < expected.size() ? expected[i] : -1;
        std::snprintf(line, sizeof(line), "       [%3zu] actual=%3d expected=%3d%s\n", i, a, e,
                      a != e ? "   <-- diff" : "");
        oss << line;
    }
    throw Failure{oss.str()};
}

}  // namespace chros_test

#endif
