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

inline __device__ __host__ float3 Cross(const float3& v1, const float3& v2) {
    return make_float3(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

inline __device__ __host__ float Dot(const float3& v1, const float3& v2) {
    return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

// Get vector 2-norm
inline __device__ __host__ float Length(const float3& v) {
    return sqrtf(Dot(v, v));
}

// Multiply a * v
inline __device__ __host__ float3 operator*(const float& a, const float3& v) {
    return make_float3(a * v.x, a * v.y, a * v.z);
}

// Multiply a * v
inline __device__ __host__ float3 operator*(const float3& v, const float& a) {
    return make_float3(a * v.x, a * v.y, a * v.z);
}

// Divide v / a
inline __device__ __host__ float3 operator/(const float3& v, const float& a) {
    return make_float3(v.x / a, v.y / a, v.z / a);
}

// v1 - v2
inline __device__ __host__ float3 operator-(const float3& v1, const float3& v2) {
    return make_float3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
}

// subtract a scalar "a" from each element
inline __device__ __host__ float3 operator-(const float3& v1, const float& a) {
    return make_float3(v1.x - a, v1.y - a, v1.z - a);
}

// v1 + v2
inline __device__ __host__ float3 operator+(const float3& v1, const float3& v2) {
    return make_float3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
}

/// This utility function returns the normal to the triangular face defined by
/// the vertices A, B, and C. The face is assumed to be non-degenerate.
/// Note that order of vertices is important!
inline __device__ __host__ float3 face_normal(const float3& A, const float3& B, const float3& C) {
    float3 nVec = Cross(B - A, C - A);
    return nVec / Length(nVec);
}

inline __device__ __host__ unsigned int hashmapTagGenerator(unsigned int seed) {
    /// Generates a "random" hashtag using a Park-Miller RNG using only 32-bit arithmetic. Care was taken here to avoid
    /// overflow. This is deterministic: the same seed will generate the same hashmap tag.
    /// Source: https://en.wikipedia.org/wiki/Lehmer_random_number_generator
    const unsigned int N = 0x7fffffff;
    const unsigned int G = 48271u;
    unsigned int div = seed / (N / G);  /// max : 2,147,483,646 / 44,488 = 48,271
    unsigned int rem = seed % (N / G);  /// max : 2,147,483,646 % 44,488 = 44,487
    unsigned int a = rem * G;            /// max : 44,487 * 48,271 = 2,147,431,977
    unsigned int b = div * (N % G);      /// max : 48,271 * 3,399 = 164,073,129
    return (a > b) ? (a - b) : (a + (N - b));
}
