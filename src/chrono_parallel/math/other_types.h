// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
//
// Description: definition of other types such as int3 and int2
// =============================================================================

#pragma once

#include "chrono_parallel/math/real3.h"  // for real3

#define S2 _make_short2
#define U3 _make_uint3
#define I2 _make_int2

typedef unsigned int uint;

namespace chrono {

struct bool2 {
    bool x, y;
    bool2() : x(0), y(0) {}
    bool2(bool a, bool b) : x(a), y(b) {}
};

struct short2 {
    short x, y;
};

class int2 {
  public:
    inline int2() : x(0), y(0) {}
    inline int2(int a) : x(a), y(a) {}
    inline int2(int a, int b, int c) : x(a), y(b) {}
    inline int2(const int2& v) : x(v.x), y(v.y) {}
    inline int2(const real2& v) : x(v.x), y(v.y) {}
    inline int operator[](unsigned int i) const { return array[i]; }
    inline int& operator[](unsigned int i) { return array[i]; }
    inline int2& operator=(const int2& rhs) {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }
    inline int2& operator=(const real2& rhs) {
        x = rhs.x;
        y = rhs.y;
        return *this;
    }
    union {
        int array[2];
        struct {
            int x, y;
        };
    };
};

class int3 {
  public:
    inline int3() : x(0), y(0), z(0), w(0) {}
    inline int3(int a) : x(a), y(a), z(a), w(0) {}
    inline int3(int a, int b, int c) : x(a), y(b), z(c), w(0) {}
    inline int3(const int3& v) : x(v.x), y(v.y), z(v.z), w(0) {}
    inline int3(const real3& v) : x(v.x), y(v.y), z(v.z), w(0) {}
    inline int operator[](unsigned int i) const { return array[i]; }
    inline int& operator[](unsigned int i) { return array[i]; }
    inline int3(__m128i m) { _mm_storeu_si128((__m128i*)&array[0], m); }
    inline operator __m128i() const { return _mm_loadu_si128((__m128i*)&array[0]); }
    inline int3& operator=(const __m128i& rhs) {
        _mm_storeu_si128((__m128i*)&array[0], rhs);
        return *this;
    }
    inline int3& operator=(const int3& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }
    inline int3& operator=(const real3& rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }
    union {
        int array[4];
        struct {
            int x, y, z, w;
        };
    };
};

struct int4 {
    int x, y, z, w;
};

struct uint4 {
    unsigned int x, y, z, w;
};
int3 operator-(const int3& a, const int3& b);
int3 operator-(const int3& a, const int& b);
int3 operator+(const int3& a, const int3& b);
int3 operator+(const int3& a, const int& b);
int3 Clamp(const int3& a, const int3& clamp_min, const int3& clamp_max);

struct uint3 {
    unsigned int x, y, z;
};

static inline short2 _make_short2(const short& a, const short& b) {
    short2 t;
    t.x = a;
    t.y = b;
    return t;
}

static inline int2 _make_int2(const int& a, const int& b) {
    int2 t;
    t.x = a;
    t.y = b;
    return t;
}

static inline uint3 _make_uint3(const real3& a) {
    uint3 t;
    t.x = uint(a.x);
    t.y = uint(a.y);
    t.z = uint(a.z);
    return t;
}

static inline uint3 _make_uint3(const uint& a, const uint& b, const uint& c) {
    uint3 t;
    t.x = a;
    t.y = b;
    t.z = c;
    return t;
}

static inline uint4 _make_uint4(const uint& a, const uint& b, const uint& c, const uint& d) {
    uint4 t;
    t.x = a;
    t.y = b;
    t.z = c;
    t.w = d;
    return t;
}
static inline uint4 Sort(const uint4& a) {
    uint4 t = a;
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
static inline uint3 Sort(const uint3& a) {
    uint3 t = a;
    SwapIfGreater(t.x, t.y);
    SwapIfGreater(t.x, t.z);
    SwapIfGreater(t.y, t.z);
    return t;
}
static inline uint3 operator-(const uint3& a, const uint3& b) {
    return U3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline std::ostream& operator<<(std::ostream& out, const int2& a) {
    out << "[" << a.x << ", " << a.y << "]" << std::endl;
    return out;
}
static inline std::ostream& operator<<(std::ostream& out, const int3& a) {
    out << "[" << a.x << ", " << a.y << ", " << a.z << "]" << std::endl;
    return out;
}

static bool operator<(const uint3& a, const uint3& b) {
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
static bool operator>(const uint3& a, const uint3& b) {
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
static bool operator==(const uint3& lhs, const uint3& rhs) {
    return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
}
}
