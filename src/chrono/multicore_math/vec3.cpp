// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar, Radu Serban
// =============================================================================

#include "chrono/multicore_math/simd.h"
#include "chrono/multicore_math/types.h"

#if defined(USE_SSE)
    #include "chrono/multicore_math/simd_sse.h"
#elif defined(USE_AVX)
    #include "chrono/multicore_math/simd_avx.h"
#else
    #include "chrono/multicore_math/simd_non.h"
#endif

namespace chrono {

vec3 operator-(const vec3& a, const vec3& b) {
    return simd::Sub(a, b);
}

vec3 operator-(const vec3& a, const int& b) {
    return simd::Sub(a, simd::Set(b));
}

vec3 operator+(const vec3& a, const vec3& b) {
    return simd::Add(a, b);
}

vec3 operator+(const vec3& a, const int& b) {
    return simd::Add(a, simd::Set(b));
}

vec3 Clamp(const vec3& a, const vec3& clamp_min, const vec3& clamp_max) {
    return simd::Max(clamp_min, simd::Min(a, clamp_max));
}

vec3 Max(const vec3& a, const vec3& b) {
    return simd::Max(a, b);
}

vec3 Min(const vec3& a, const vec3& b) {
    return simd::Min(a, b);
}

}  // namespace chrono
