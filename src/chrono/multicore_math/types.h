// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Description: definition of types for multicore math
// =============================================================================

#pragma once

#include "chrono/multicore_math/real.h"
#include "chrono/multicore_math/real2.h"
#include "chrono/multicore_math/real3.h"
#include "chrono/multicore_math/real4.h"

#define S2 _make_short2
#define U3 _make_uvec3
#define I2 _make_vec2

typedef unsigned int uint;

namespace chrono {

/// @addtogroup chrono_mc_math
/// @{

struct bool2 {
    bool x, y;
    bool2() : x(false), y(false) {}
    bool2(bool a, bool b) : x(a), y(b) {}
};

struct short2 {
    short x, y;
    short2() : x(0), y(0) {}
    short2(short a, short b) : x(a), y(b) {}
};

class ChApi vec2 {
  public:
    inline vec2() : x(0), y(0) {}
    inline vec2(int a) : x(a), y(a) {}
    inline vec2(int a, int b) : x(a), y(b) {}
    inline vec2(const vec2& v) : x(v.x), y(v.y) {}
    inline vec2(const real2& v) : x(int(v.x)), y(int(v.y)) {}
    inline int operator[](unsigned int i) const { return array[i]; }
    inline int& operator[](unsigned int i) { return array[i]; }
    inline vec2& operator=(const vec2& rhs) {
        x = int(rhs.x);
        y = int(rhs.y);
        return *this;
    }
    inline vec2& operator=(const real2& rhs) {
        x = int(rhs.x);
        y = int(rhs.y);
        return *this;
    }
    union {
        int array[2];
        struct {
            int x, y;
        };
    };
};

class ChApi vec3 {
  public:
    inline vec3() : x(0), y(0), z(0), w(0) {}
    inline vec3(int a) : x(a), y(a), z(a), w(0) {}
    inline vec3(int a, int b, int c) : x(a), y(b), z(c), w(0) {}
    inline vec3(const vec3& v) : x(v.x), y(v.y), z(v.z), w(0) {}
    inline vec3(const real3& v) : x(int(v.x)), y(int(v.y)), z(int(v.z)), w(0) {}
    inline int operator[](unsigned int i) const { return array[i]; }
    inline int& operator[](unsigned int i) { return array[i]; }

#if defined(USE_SSE) || defined(USE_AVX)
    inline vec3(__m128i m) { _mm_storeu_si128((__m128i*)&array[0], m); }
    inline operator __m128i() const { return _mm_loadu_si128((__m128i*)&array[0]); }
    inline vec3& operator=(const __m128i& rhs) {
        _mm_storeu_si128((__m128i*)&array[0], rhs);
        return *this;
    }
#endif

    inline vec3& operator=(const vec3& rhs) {
        x = int(rhs.x);
        y = int(rhs.y);
        z = int(rhs.z);
        return *this;
    }
    inline vec3& operator=(const real3& rhs) {
        x = int(rhs.x);
        y = int(rhs.y);
        z = int(rhs.z);
        return *this;
    }
    union {
        int array[4];
        struct {
            int x, y, z, w;
        };
    };
};

class ChApi real3_int {
  public:
    real3_int() {}
    real3_int(real3 a, int b) : v(a), i(b) {}

    real3 v;
    int i;
};

ChApi vec3 operator-(const vec3& a, const vec3& b);
ChApi vec3 operator-(const vec3& a, const int& b);
ChApi vec3 operator+(const vec3& a, const vec3& b);
ChApi vec3 operator+(const vec3& a, const int& b);
ChApi vec3 Clamp(const vec3& a, const vec3& clamp_min, const vec3& clamp_max);
ChApi vec3 Max(const vec3& a, const vec3& b);
ChApi vec3 Min(const vec3& a, const vec3& b);

struct vec4 {
    int x, y, z, w;
};

struct uvec4 {
    unsigned int x, y, z, w;
};

struct uvec3 {
    unsigned int x, y, z;
};

static inline short2 _make_short2(const short& a, const short& b) {
    short2 t;
    t.x = a;
    t.y = b;
    return t;
}

static inline vec2 _make_vec2(const int& a, const int& b) {
    vec2 t;
    t.x = a;
    t.y = b;
    return t;
}

static inline uvec3 _make_uvec3(const real3& a) {
    uvec3 t;
    t.x = uint(a.x);
    t.y = uint(a.y);
    t.z = uint(a.z);
    return t;
}

static inline uvec3 _make_uvec3(const uint& a, const uint& b, const uint& c) {
    uvec3 t;
    t.x = a;
    t.y = b;
    t.z = c;
    return t;
}

static inline uvec4 _make_uvec4(const uint& a, const uint& b, const uint& c, const uint& d) {
    uvec4 t;
    t.x = a;
    t.y = b;
    t.z = c;
    t.w = d;
    return t;
}

static inline uvec4 Sort(const uvec4& a) {
    uvec4 t = a;
    if (t.x > t.w) {
        Swap(t.x, t.w);
    }
    if (t.x > t.z) {
        Swap(t.x, t.z);
    }
    if (t.x > t.y) {
        Swap(t.x, t.y);
    }

    if (t.y > t.w) {
        Swap(t.y, t.w);
    }
    if (t.y > t.z) {
        Swap(t.y, t.z);
    }

    if (t.z > t.w) {
        Swap(t.z, t.w);
    }
    return t;
}

static inline uvec3 Sort(const uvec3& a) {
    uvec3 t = a;
    SwapIfGreater(t.x, t.y);
    SwapIfGreater(t.x, t.z);
    SwapIfGreater(t.y, t.z);
    return t;
}

static inline uvec3 operator-(const uvec3& a, const uvec3& b) {
    return U3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline std::ostream& operator<<(std::ostream& out, const vec2& a) {
    out << "[" << a.x << ", " << a.y << "]" << std::endl;
    return out;
}

static inline std::ostream& operator<<(std::ostream& out, const vec3& a) {
    out << "[" << a.x << ", " << a.y << ", " << a.z << "]" << std::endl;
    return out;
}

static bool operator<(const uvec3& a, const uvec3& b) {
    if (a.x < b.x) {
        return true;
    }
    if (b.x < a.x) {
        return false;
    }
    if (a.y < b.y) {
        return true;
    }
    if (b.y < a.y) {
        return false;
    }
    if (a.z < b.z) {
        return true;
    }
    if (b.z < a.z) {
        return false;
    }
    return false;
}

static bool operator>(const uvec3& a, const uvec3& b) {
    if (a.x > b.x) {
        return true;
    }
    if (b.x > a.x) {
        return false;
    }
    if (a.y > b.y) {
        return true;
    }
    if (b.y > a.y) {
        return false;
    }
    if (a.z > b.z) {
        return true;
    }
    if (b.z > a.z) {
        return false;
    }
    return false;
}

static bool operator==(const uvec3& lhs, const uvec3& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
}

/// @} chrono_mc_math

}  // end namespace chrono
