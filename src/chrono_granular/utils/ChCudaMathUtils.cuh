// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Dan Negrut, Conlain Kelly
// =============================================================================

#pragma once

#define MIN(a, b) (a < b) ? a : b
#define MAX(a, b) (a > b) ? a : b

inline __device__ double3 Cross(const double3& v1, const double3& v2) {
    return make_double3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

inline __device__ double Dot(const double3& v1, const double3& v2) {
    return __dadd_ru(__dadd_ru(__dmul_ru(v1.x, v2.x), __dmul_ru(v1.y, v2.y)), __dmul_ru(v1.z, v2.z));
}

// Get vector 2-norm
inline __device__ double Length(const double3& v) {
    return __dsqrt_ru(Dot(v, v));
}

// Multiply a * v
inline __device__ double3 operator*(const double& a, const double3& v) {
    return make_double3(__dmul_ru(a, v.x), __dmul_ru(a, v.y), __dmul_ru(a, v.z));
}

// Divide v / a
inline __device__ double3 operator/(const double3& v, const double& a) {
    return make_double3(v.x / a, v.y / a, v.z / a);
}

// v1 - v2
inline __device__ double3 operator-(const double3& v1, const double3& v2) {
    return make_double3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

// v1 + v2
inline __device__ double3 operator+(const double3& v1, const double3& v2) {
    return make_double3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

/// This utility function returns the normal to the triangular face defined by
/// the vertices A, B, and C. The face is assumed to be non-degenerate.
/// Note that order of vertices is important!
inline __device__ double3 face_normal(const double3& A, const double3& B, const double3& C) {
    double3 nVec = Cross(B - A, C - A);
    return nVec / Length(nVec);
}

inline __device__ unsigned int hashmapTagGenerator(unsigned int seed) {
    /// Generates a "random" hashtag empoloying a Park-Miller RNG using only 32-bit arithmetic. Care was taken here to
    /// avoid overflow. This is deterministic: the same seed will generate the same hashmap tag. Source:
    /// https://en.wikipedia.org/wiki/Lehmer_random_number_generator
    const unsigned int N = 0x7fffffff;
    const unsigned int G = 48271u;
    unsigned int div = seed / (N / G);  /// max : 2,147,483,646 / 44,488 = 48,271
    unsigned int rem = seed % (N / G);  /// max : 2,147,483,646 % 44,488 = 44,487
    unsigned int a = rem * G;           /// max : 44,487 * 48,271 = 2,147,431,977
    unsigned int b = div * (N % G);     /// max : 48,271 * 3,399 = 164,073,129
    return (a > b) ? (a - b) : (a + (N - b));
}
