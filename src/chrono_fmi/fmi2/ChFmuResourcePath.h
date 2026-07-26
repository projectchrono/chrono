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
// Conversion of an FMI 2.0 resource location (a URI) to a local filesystem path.
//
// Kept free of any fmu-forge dependency so that it can be included, and tested,
// on its own.
//
// =============================================================================

#ifndef CH_FMU2_RESOURCE_PATH_H
#define CH_FMU2_RESOURCE_PATH_H

#include <string>

namespace chrono {
namespace fmi2 {

/// @addtogroup chrono_fmi2
/// @{

namespace detail {

// ASCII-only character classification. The <cctype> functions are locale-dependent, which a URI parser must not be:
// under a Turkish locale, for instance, tolower('I') is the dotless 'i', so "FILE:" would not fold to "file:".

inline char AsciiToLower(char c) {
    return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c;
}

inline bool AsciiIsAlpha(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

/// Return the value of a hexadecimal digit, or -1 if the character is not one.
inline int AsciiHexValue(char c) {
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

inline std::string AsciiToLower(const std::string& s) {
    std::string out;
    out.reserve(s.length());
    for (char c : s)
        out.push_back(AsciiToLower(c));
    return out;
}

}  // namespace detail

/// Convert an FMI 2.0 resource location to a local filesystem path.
/// The FMI 2.0 standard specifies that the `fmuResourceLocation` argument of `fmi2Instantiate` is a URI (RFC 3986)
/// pointing to the 'resources' directory of the unpacked FMU, not a plain path. Note that simply removing a
/// fixed-length "file:///" prefix is not correct: it discards the leading slash of a POSIX absolute path, turning it
/// into a relative one.
///
/// The forms accepted, per RFC 8089, are:
/// <pre>
///     file:///tmp/fmu/resources       ->  /tmp/fmu/resources      empty authority, POSIX absolute path
///     file:///C:/temp/fmu/resources   ->  C:/temp/fmu/resources   empty authority, drive letter (Windows only)
///     file://localhost/tmp/resources  ->  /tmp/resources          "localhost" authority, matched case-insensitively
///     file:/tmp/fmu/resources         ->  /tmp/fmu/resources      RFC 8089 minimal form, authority omitted
///     file:////tmp/fmu/resources      ->  /tmp/fmu/resources      repeated leading slashes collapsed (see below)
///     /tmp/fmu/resources              ->  /tmp/fmu/resources      plain path, for importers that pass one
/// </pre>
///
/// The four-slash form is not standard, but Chrono's own FMU importer produces it by concatenating "file:///" with a
/// directory that is already absolute, so it must keep working.
///
/// Interpretation is deliberately platform-dependent, because file URIs are: the drive-letter rule applies only on
/// Windows, since "/C:/temp" is a perfectly legal POSIX path naming a directory called "C:".
///
/// A URI naming a remote host (a non-empty authority other than "localhost") does not denote a local path. On Windows
/// it is converted to the equivalent UNC path; elsewhere the input is returned unchanged, so that the caller fails to
/// open it and reports the full URI, rather than being silently redirected to an unrelated local path (on Linux,
/// "//host/share" would otherwise resolve as "/host/share").
///
/// Percent-encoded octets are decoded. A percent sign not followed by two hexadecimal digits is left alone. Decoding
/// is applied only when the location actually carried the "file:" scheme, so a plain path containing a literal '%' is
/// preserved. Any trailing separator is removed, so that callers can uniformly append "/<filename>".
inline std::string GetResourcesPath(const char* resource_location) {
    if (!resource_location)
        return "";

    const std::string original(resource_location);
    std::string path = original;

    // Look for a "file:" scheme. Anything else is treated as a plain path; in particular, a Windows path such as
    // "C:/temp" must not be mistaken for a URI with scheme "C".
    if (detail::AsciiToLower(path.substr(0, 5)) != "file:") {
        while (path.length() > 1 && (path.back() == '/' || path.back() == '\\'))
            path.pop_back();
        return path;
    }

    path.erase(0, 5);

    // Remove the authority component, if present. RFC 8089 allows it to be empty ("file:///path") or "localhost";
    // either denotes the local machine. Host names are case-insensitive.
    if (path.compare(0, 2, "//") == 0) {
        const std::string rest = path.substr(2);
        const auto slash = rest.find('/');
        const std::string authority = detail::AsciiToLower(rest.substr(0, slash));

        if (authority.empty() || authority == "localhost") {
            path = (slash == std::string::npos) ? "" : rest.substr(slash);
        } else {
#if defined(_WIN32)
            // A remote host maps to a UNC path.
            return "//" + rest;
#else
            // No local interpretation exists. Return the location untouched so that it fails visibly.
            return original;
#endif
        }
    }

    // Collapse repeated leading slashes (the Chrono four-slash form).
    auto first = path.find_first_not_of('/');
    if (first == std::string::npos)
        first = path.length();
    if (first > 1)
        path.erase(0, first - 1);

#if defined(_WIN32)
    // On Windows the drive letter follows the leading slash ("/C:/temp"); remove that slash. The legacy "|" drive
    // separator is also accepted.
    if (path.length() >= 3 && path[0] == '/' && detail::AsciiIsAlpha(path[1]) && (path[2] == ':' || path[2] == '|')) {
        path.erase(0, 1);
        path[1] = ':';
    }
#endif

    // Decode percent-encoded octets.
    std::string decoded;
    decoded.reserve(path.length());
    for (size_t i = 0; i < path.length(); i++) {
        int hi = -1;
        int lo = -1;
        if (path[i] == '%' && i + 2 < path.length() && (hi = detail::AsciiHexValue(path[i + 1])) >= 0 &&
            (lo = detail::AsciiHexValue(path[i + 2])) >= 0) {
            // Round-trip through unsigned char so that octets above 0x7F survive on signed-char platforms.
            decoded.push_back(static_cast<char>(static_cast<unsigned char>(hi * 16 + lo)));
            i += 2;
        } else {
            decoded.push_back(path[i]);
        }
    }
    path = decoded;

    // Drop a trailing separator so that callers can uniformly append "/<filename>".
    while (path.length() > 1 && (path.back() == '/' || path.back() == '\\'))
        path.pop_back();

    return path;
}

/// @} chrono_fmi2

}  // namespace fmi2
}  // namespace chrono

#endif
