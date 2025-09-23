// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Conlain Kelly, Nic Olsen, Dan Negrut
// =============================================================================

#pragma once

#include <stdio.h>
#include <stdlib.h>

#define CHDEM_ERROR(...)                  \
    {                                     \
        printf(__VA_ARGS__);              \
        printf(__func__);                 \
        printf("\n: EXITING DEM SIM.\n"); \
        exit(1);                          \
    }

// defined in ChDem.cpp
extern size_t gran_approx_bytes_used;

// Add verbose checks easily
#define TRACK_VECTOR_RESIZE(vec, newsize, name, val)                                                             \
    {                                                                                                            \
        size_t item_size = sizeof(decltype(vec)::value_type);                                                    \
        size_t old_size = vec.size();                                                                            \
        vec.resize(newsize, val);                                                                                \
        size_t new_size = vec.size();                                                                            \
        size_t byte_delta = item_size * (new_size - old_size);                                                   \
        gran_approx_bytes_used += byte_delta;                                                                    \
        INFO_PRINTF("Resizing vector %s, old size %zu, new size %zu, byte delta %s\n", name, old_size, new_size, \
                    pretty_format_bytes(byte_delta).c_str());                                                    \
    }

inline std::string pretty_format_bytes(size_t bytes) {
    // set up byte prefixes
    constexpr size_t KIBI = 1024;
    constexpr size_t MEBI = KIBI * KIBI;
    constexpr size_t GIBI = KIBI * KIBI * KIBI;
    float gibival = float(bytes) / GIBI;
    float mebival = float(bytes) / MEBI;
    float kibival = float(bytes) / KIBI;
    std::stringstream ret;
    if (gibival > 1) {
        ret << gibival << " GiB";
    } else if (mebival > 1) {
        ret << mebival << " MiB";
    } else if (kibival > 1) {
        ret << kibival << " KiB";
    } else {
        ret << bytes << " B";
    }
    return ret.str();
}

// calculate effective youngs modulus and shear modulus
inline void materialPropertyCombine(double Y1, double Y2, double nu1, double nu2, double& E_eff, double& G_eff) {
    double invE = (1 - nu1 * nu1) / Y1 + (1 - nu2 * nu2) / Y2;
    E_eff = 1. / invE;
    double invG = 2 * (2 - nu1) * (1 + nu1) / Y1 + 2 * (2 - nu2) * (1 + nu2) / Y2;
    G_eff = 1. / invG;
}