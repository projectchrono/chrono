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
// Unit test for decoding the FMU resource location.
//
// The FMI standards and actual importers disagree on the form of the resource location handed to an FMU at
// instantiation, so fmu_forge::ResourceLocationToPath has to accept all of them. Regression test for
// projectchrono/chrono#762, where a conforming "file:///abs/path" from FMPy lost its root slash on POSIX and the
// FMU then looked for its resources at a relative path.
//
// Every expectation here is platform independent: the decoder has no platform-specific branches, and a location
// carrying no "file:" scheme is already a path and is returned verbatim on any OS.
// =============================================================================

#include <string>

#include "FmuToolsResourceLocation.h"

#include "gtest/gtest.h"

using fmu_forge::ResourceLocationToPath;

// -----------------------------------------------------------------------------
// The standard RFC 8089 form, emitted by FMPy and by fmusim. Losing the root slash on the POSIX case is issue #762.

TEST(FmuResourceLocation, StandardFileURI) {
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/tmpX/resources"), "/tmp/tmpX/resources");
    EXPECT_EQ(ResourceLocationToPath("file:///C:/Users/dn/Temp/tmpX/resources"), "C:/Users/dn/Temp/tmpX/resources");
}

// -----------------------------------------------------------------------------
// The four-slash form that fmu_forge::FmuUnit::Instantiate emits on POSIX, where m_directory is
// already rooted. Extra leading slashes after "file:" are syntactically permitted, so this is unusual
// practice rather than invalid; either way FMUs in circulation depend on it, so it must keep resolving
// to the same absolute path as the standard three-slash spelling.

TEST(FmuResourceLocation, LegacyFourSlashURI) {
    EXPECT_EQ(ResourceLocationToPath("file:////tmp/_fmu_temp/resources"), "/tmp/_fmu_temp/resources");
    EXPECT_EQ(ResourceLocationToPath("file:///C:/Users/dn/Temp/_fmu_temp/resources"),
              "C:/Users/dn/Temp/_fmu_temp/resources");
}

// -----------------------------------------------------------------------------
// Authority component: empty or "localhost", or absent entirely. A drive letter sitting where an authority would
// go must not be eaten as one.

TEST(FmuResourceLocation, AuthorityForms) {
    EXPECT_EQ(ResourceLocationToPath("file://localhost/tmp/x/resources"), "/tmp/x/resources");
    EXPECT_EQ(ResourceLocationToPath("file://LocalHost/tmp/x/resources"), "/tmp/x/resources");  // host is case-insensitive
    EXPECT_EQ(ResourceLocationToPath("file:/tmp/x/resources"), "/tmp/x/resources");
    EXPECT_EQ(ResourceLocationToPath("file://C:/x/resources"), "C:/x/resources");
}

// -----------------------------------------------------------------------------
// A non-local authority names the host holding the file (RFC 8089). Dropping it would silently turn a remote
// location into a different local path, so it is preserved as a UNC path.

TEST(FmuResourceLocation, NonLocalAuthorityBecomesUNC) {
    EXPECT_EQ(ResourceLocationToPath("file://server/share/resources"), "//server/share/resources");
    EXPECT_EQ(ResourceLocationToPath("file://server.example.com/share/res"), "//server.example.com/share/res");
}

// -----------------------------------------------------------------------------
// URI schemes are case-insensitive (RFC 3986, section 3.1).

TEST(FmuResourceLocation, SchemeIsCaseInsensitive) {
    EXPECT_EQ(ResourceLocationToPath("FILE:///tmp/x/resources"), "/tmp/x/resources");
    EXPECT_EQ(ResourceLocationToPath("FiLe:///tmp/x/resources"), "/tmp/x/resources");
    EXPECT_EQ(ResourceLocationToPath("FILE:///C:/x/resources"), "C:/x/resources");
    // A scheme that merely starts with the same letters is not "file:".
    EXPECT_EQ(ResourceLocationToPath("filex:///tmp/x"), "filex:///tmp/x");
}

// -----------------------------------------------------------------------------
// Degenerate URIs: a "file:" URI must always yield a rooted path, never a stray "//" or an empty string
// (an empty return means "could not decode" and sends the caller to its fallback).

TEST(FmuResourceLocation, DegenerateSlashOnlyURIs) {
    EXPECT_EQ(ResourceLocationToPath("file:/"), "/");
    EXPECT_EQ(ResourceLocationToPath("file://"), "/");
    EXPECT_EQ(ResourceLocationToPath("file:///"), "/");
    EXPECT_EQ(ResourceLocationToPath("file:////"), "/");
    EXPECT_EQ(ResourceLocationToPath("file://///"), "/");
}

// -----------------------------------------------------------------------------
// Percent-encoded octets, which importers produce for any path containing a space.

TEST(FmuResourceLocation, PercentEncoding) {
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/my%20dir/resources"), "/tmp/my dir/resources");
    EXPECT_EQ(ResourceLocationToPath("file:///C:/Program%20Files/res"), "C:/Program Files/res");
    // A '%' not introducing two hex digits is passed through rather than mangled.
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/100%/res"), "/tmp/100%/res");
    // Octets above 0x7F must survive decoding: bytes are bytes, and the hex test must not depend on
    // locale or on char signedness. "caf%C3%A9" is UTF-8 for "café".
    const std::string cafe = std::string("/tmp/caf") + char(0xC3) + char(0xA9) + "/res";
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/caf%C3%A9/res"), cafe);
    // A raw (unescaped) high byte is left alone, and must not be mistaken for a drive letter.
    const std::string raw = std::string("/tmp/") + char(0xC3) + "/res";
    EXPECT_EQ(ResourceLocationToPath(std::string("file:///tmp/") + char(0xC3) + "/res"), raw);
}

// -----------------------------------------------------------------------------
// An unescaped '?' starts the query and '#' starts the fragment; neither is part of the path (RFC 3986, section 3.3).
// The regex this decoder replaced captured ([^#\?]+), i.e. it stopped at both, so keeping them would be a regression:
// the expected values below are the ones the previous implementation produced.

TEST(FmuResourceLocation, QueryAndFragmentAreNotPartOfThePath) {
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/res?foo=1"), "/tmp/res");
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/res#bar"), "/tmp/res");
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/res?foo=1#bar"), "/tmp/res");
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/my%20dir/res?v=2"), "/tmp/my dir/res");
    EXPECT_EQ(ResourceLocationToPath("file://localhost/tmp/res?v=2"), "/tmp/res");
    // A delimiter immediately after the path leaves the path intact.
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/res/?v=2"), "/tmp/res/");
    // Escaped forms are ordinary characters in a file name and must survive.
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/what%3F/res"), "/tmp/what?/res");
    EXPECT_EQ(ResourceLocationToPath("file:///tmp/hash%23tag/res"), "/tmp/hash#tag/res");
    // A plain path is not a URI, so a '?' in it is just a character.
    EXPECT_EQ(ResourceLocationToPath("/tmp/res?foo=1"), "/tmp/res?foo=1");
}

// -----------------------------------------------------------------------------
// A relative reference has a relative path by definition; the decoder must not invent a root for it.

TEST(FmuResourceLocation, RelativeReferenceStaysRelative) {
    EXPECT_EQ(ResourceLocationToPath("file:resources/Vehicle.json"), "resources/Vehicle.json");
    EXPECT_EQ(ResourceLocationToPath("file:resources"), "resources");
}

// -----------------------------------------------------------------------------
// A location with no scheme is a path already: what FMI 3.0 specifies for resourcePath, and what
// ChExternalFmu passes when given an explicit resources directory. Returned verbatim, so a literal '%' survives
// and a UNC path is never mistaken for a URI authority.

TEST(FmuResourceLocation, PlainPathsReturnedVerbatim) {
    EXPECT_EQ(ResourceLocationToPath("/home/x/res"), "/home/x/res");
    EXPECT_EQ(ResourceLocationToPath("C:/x/res"), "C:/x/res");
    EXPECT_EQ(ResourceLocationToPath("C:\\x\\res"), "C:\\x\\res");
    EXPECT_EQ(ResourceLocationToPath("\\\\server\\share\\res"), "\\\\server\\share\\res");
    EXPECT_EQ(ResourceLocationToPath("resources"), "resources");
    EXPECT_EQ(ResourceLocationToPath("/home/x/100%/res"), "/home/x/100%/res");
}

// -----------------------------------------------------------------------------
// An empty result is the only failure mode, and is what makes the caller fall back to a library-relative guess.

TEST(FmuResourceLocation, EmptyInput) {
    EXPECT_EQ(ResourceLocationToPath(""), "");
}

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// JoinPath backs FmuComponentBase::GetResourcePath(), which is how a concrete FMU appends a file name to the
// resources directory without knowing whether that directory ends in a separator. That independence is the point:
// it means a future change to how the resources location is stored cannot silently produce "resourcesVehicle.json".

TEST(FmuResourceLocation, JoinPathIsSeparatorAgnostic) {
    using fmu_forge::JoinPath;
    // With and without a trailing separator, same answer.
    EXPECT_EQ(JoinPath("/tmp/res/", "Vehicle.json"), "/tmp/res/Vehicle.json");
    EXPECT_EQ(JoinPath("/tmp/res", "Vehicle.json"), "/tmp/res/Vehicle.json");
    // Windows separators are honored rather than doubled.
    EXPECT_EQ(JoinPath("C:\\tmp\\res\\", "Vehicle.json"), "C:\\tmp\\res\\Vehicle.json");
    EXPECT_EQ(JoinPath("C:/tmp/res", "Vehicle.json"), "C:/tmp/res/Vehicle.json");
    // Degenerate arguments concatenate rather than inventing a separator.
    EXPECT_EQ(JoinPath("", "Vehicle.json"), "Vehicle.json");
    EXPECT_EQ(JoinPath("/tmp/res/", ""), "/tmp/res/");
    EXPECT_EQ(JoinPath("", ""), "");
    // A decoded resource location fed straight in, as the FMUs do.
    EXPECT_EQ(JoinPath(ResourceLocationToPath("file:///tmp/x/resources"), "Vehicle.json"),
              "/tmp/x/resources/Vehicle.json");
}

// -----------------------------------------------------------------------------

TEST(FmuResourceLocation, PercentDecode) {
    EXPECT_EQ(fmu_forge::PercentDecode("a%20b"), "a b");
    EXPECT_EQ(fmu_forge::PercentDecode("%2Fa%2fb"), "/a/b");  // upper and lower case hex
    EXPECT_EQ(fmu_forge::PercentDecode("100%"), "100%");      // truncated escape
    EXPECT_EQ(fmu_forge::PercentDecode("%zz"), "%zz");        // not hex
    EXPECT_EQ(fmu_forge::PercentDecode("%2"), "%2");          // one digit short
    EXPECT_EQ(fmu_forge::PercentDecode("nothing"), "nothing");
}
