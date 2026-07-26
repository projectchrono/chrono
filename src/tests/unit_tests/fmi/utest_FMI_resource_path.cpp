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
// Authors: Radu Serban
// =============================================================================
//
// Test conversion of an FMI 2.0 resource location (a URI) to a local filesystem
// path.
//
// The FMI 2.0 standard specifies that the fmuResourceLocation argument of
// fmi2Instantiate is a URI, not a plain path. Removing a fixed-length "file:///"
// prefix works on Windows, where the drive letter follows the third slash, but
// discards the leading slash of a POSIX absolute path and so silently produces a
// relative one (issue #762).
//
// Interpretation of a file URI is platform-dependent, so the drive-letter and
// remote-host expectations below are guarded accordingly.
//
// =============================================================================

#include <clocale>
#include <string>

#include "gtest/gtest.h"

#include "chrono_fmi/fmi2/ChFmuResourcePath.h"

using namespace chrono::fmi2;

// Locations emitted by standard-conforming importers (FMPy, Dymola, OpenModelica):
// an empty authority, so exactly three slashes.
TEST(FmiResourcePath, ConformingImporter) {
    // The case reported in issue #762. A fixed 8-character chop yields "tmp/tmpyr1j7rvs/resources".
    EXPECT_EQ(GetResourcesPath("file:///tmp/tmpyr1j7rvs/resources"), "/tmp/tmpyr1j7rvs/resources");
    EXPECT_EQ(GetResourcesPath("file:///home/user/unpacked/resources"), "/home/user/unpacked/resources");

    // RFC 8089 also permits the minimal form, with the authority omitted entirely.
    EXPECT_EQ(GetResourcesPath("file:/tmp/fmu/resources"), "/tmp/fmu/resources");
}

// Chrono's own importer builds the location by concatenating "file:///" with a directory that is already absolute,
// which produces four slashes on POSIX. These must keep working.
TEST(FmiResourcePath, ChronoImporter) {
    EXPECT_EQ(GetResourcesPath("file:////tmp/DEMO_OUTPUT/fmu/resources"), "/tmp/DEMO_OUTPUT/fmu/resources");
}

// The drive-letter rule is Windows-only: "/C:/temp" is a legal POSIX path naming a directory called "C:".
TEST(FmiResourcePath, WindowsDriveLetter) {
#if defined(_WIN32)
    EXPECT_EQ(GetResourcesPath("file:///C:/temp/fmu/resources"), "C:/temp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("file:///d:/work/resources"), "d:/work/resources");
    EXPECT_EQ(GetResourcesPath("file:/C:/temp/resources"), "C:/temp/resources");
    EXPECT_EQ(GetResourcesPath("file://localhost/C:/temp/resources"), "C:/temp/resources");
    EXPECT_EQ(GetResourcesPath("file:///C:/temp/DEMO_OUTPUT/fmu/resources"), "C:/temp/DEMO_OUTPUT/fmu/resources");
    // Legacy "|" drive separator, still emitted by some older tools.
    EXPECT_EQ(GetResourcesPath("file:///C|/temp/resources"), "C:/temp/resources");
    EXPECT_EQ(GetResourcesPath("file:///C:/"), "C:");
    EXPECT_EQ(GetResourcesPath("file:///C:"), "C:");
#else
    // On POSIX the path is absolute and the leading slash must be preserved.
    EXPECT_EQ(GetResourcesPath("file:///C:/temp/fmu/resources"), "/C:/temp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("file:///C|/temp/resources"), "/C|/temp/resources");
#endif
}

TEST(FmiResourcePath, Authority) {
    // An empty authority and "localhost" both denote the local machine. Host names are case-insensitive.
    EXPECT_EQ(GetResourcesPath("file://localhost/tmp/fmu/resources"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("file://LOCALHOST/tmp/fmu/resources"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("file://LocalHost/tmp/fmu/resources"), "/tmp/fmu/resources");

    // A remote host does not denote a local path.
#if defined(_WIN32)
    // Map it to the equivalent UNC path.
    EXPECT_EQ(GetResourcesPath("file://otherbox/share/resources"), "//otherbox/share/resources");
#else
    // Return it untouched so it fails visibly; "//host/share" would otherwise resolve as the local "/host/share".
    EXPECT_EQ(GetResourcesPath("file://otherbox/share/resources"), "file://otherbox/share/resources");
#endif
}

TEST(FmiResourcePath, PercentDecoding) {
    EXPECT_EQ(GetResourcesPath("file:///tmp/my%20fmu/resources"), "/tmp/my fmu/resources");
    EXPECT_EQ(GetResourcesPath("file:///tmp/a%2fb"), "/tmp/a/b");
    EXPECT_EQ(GetResourcesPath("file:///tmp/a%2Fb"), "/tmp/a/b");

    // A percent sign that does not introduce a valid escape is left alone.
    EXPECT_EQ(GetResourcesPath("file:///tmp/100%/resources"), "/tmp/100%/resources");
    EXPECT_EQ(GetResourcesPath("file:///tmp/x%"), "/tmp/x%");
    EXPECT_EQ(GetResourcesPath("file:///tmp/x%GG"), "/tmp/x%GG");
    EXPECT_EQ(GetResourcesPath("file:///tmp/x%2"), "/tmp/x%2");

    // Octets above 0x7F must survive on platforms where char is signed. "%C3%A9" is UTF-8 for 'e' with acute accent.
    const std::string eacute = GetResourcesPath("file:///tmp/caf%C3%A9/resources");
    EXPECT_EQ(eacute, std::string("/tmp/caf\xC3\xA9/resources"));
    EXPECT_EQ(static_cast<unsigned char>(GetResourcesPath("file:///%FF")[1]), 0xFFu);
}

TEST(FmiResourcePath, Scheme) {
    // The scheme is case-insensitive.
    EXPECT_EQ(GetResourcesPath("FILE:///tmp/fmu/resources"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("File:///tmp/fmu/resources"), "/tmp/fmu/resources");

    // Some importers pass a plain path instead of a URI. Such a path is returned unchanged; in particular a Windows
    // path must not be mistaken for a URI with a one-letter scheme, and no percent decoding is applied.
    EXPECT_EQ(GetResourcesPath("/tmp/fmu/resources"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("C:/temp/fmu/resources"), "C:/temp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("C:\\temp\\fmu\\resources"), "C:\\temp\\fmu\\resources");
    EXPECT_EQ(GetResourcesPath("/tmp/100%20"), "/tmp/100%20");
}

// Case folding must be locale-independent: under a Turkish locale std::tolower('I') is the dotless 'i', which would
// break scheme and host matching if <cctype> were used.
TEST(FmiResourcePath, LocaleIndependence) {
    const char* saved = std::setlocale(LC_ALL, nullptr);
    const std::string saved_locale = saved ? saved : "C";

    // Try the Turkish locales this platform might offer; skip the check if none is installed.
    const char* candidates[] = {"tr_TR.UTF-8", "tr_TR", "Turkish_Turkey.1254", "Turkish"};
    bool applied = false;
    for (const char* c : candidates) {
        if (std::setlocale(LC_ALL, c)) {
            applied = true;
            break;
        }
    }

    EXPECT_EQ(GetResourcesPath("FILE:///tmp/fmu/resources"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("file://LOCALHOST/tmp/fmu/resources"), "/tmp/fmu/resources");

    std::setlocale(LC_ALL, saved_locale.c_str());

    if (!applied)
        GTEST_SKIP() << "no Turkish locale installed; the locale-sensitivity check did not actually exercise it";
}

TEST(FmiResourcePath, TrailingSeparator) {
    // Callers append "/<filename>", so a trailing separator is removed to avoid a doubled slash.
    EXPECT_EQ(GetResourcesPath("file:///tmp/fmu/resources/"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("file:///tmp/fmu/resources///"), "/tmp/fmu/resources");
    EXPECT_EQ(GetResourcesPath("C:\\temp\\resources\\"), "C:\\temp\\resources");
    EXPECT_EQ(GetResourcesPath("file:///tmp/fmu/resources/") + "/Vehicle.json", "/tmp/fmu/resources/Vehicle.json");
}

// Malformed or truncated locations must be handled without reading out of bounds.
TEST(FmiResourcePath, Degenerate) {
    EXPECT_EQ(GetResourcesPath(nullptr), "");
    EXPECT_EQ(GetResourcesPath(""), "");
    EXPECT_EQ(GetResourcesPath("a"), "a");
    EXPECT_EQ(GetResourcesPath("/"), "/");
    EXPECT_EQ(GetResourcesPath("file:"), "");
    EXPECT_EQ(GetResourcesPath("file://"), "");
    EXPECT_EQ(GetResourcesPath("file:///"), "/");
    EXPECT_EQ(GetResourcesPath("file://////"), "/");
}
