#if defined(__CUDA_ARCH__)
#include "chrono/multicore_math/simd_non.h"
#include "chrono/multicore_math/real3.h"
#include <iostream>

namespace chrono {

CUDA_HOST_DEVICE real3 Set3(real x) {
    return real3(x);
}
CUDA_HOST_DEVICE real3 Set3(real x, real y, real z) {
    return real3(x, y, z);
}

//========================================================
CUDA_HOST_DEVICE real3 operator+(const real3& a, const real3& b) {
    return simd::Add(a, b);
}
CUDA_HOST_DEVICE real3 operator-(const real3& a, const real3& b) {
    return simd::Sub(a, b);
}
CUDA_HOST_DEVICE real3 operator*(const real3& a, const real3& b) {
    return simd::Mul(a, b);
}
CUDA_HOST_DEVICE real3 operator/(const real3& a, const real3& b) {
    return simd::Div(a, b);
}
//========================================================
CUDA_HOST_DEVICE real3 operator+(const real3& a, real b) {
    return simd::Add(a, Set3(b));
}
CUDA_HOST_DEVICE real3 operator-(const real3& a, real b) {
    return simd::Sub(a, Set3(b));
}
CUDA_HOST_DEVICE real3 operator*(const real3& a, real b) {
    return simd::Mul(a, Set3(b));
}
CUDA_HOST_DEVICE real3 operator/(const real3& a, real b) {
    return simd::Div(a, Set3(b));
}
CUDA_HOST_DEVICE real3 operator*(real lhs, const real3& rhs) {
    return simd::Mul(Set3(lhs), rhs);
}
CUDA_HOST_DEVICE real3 operator/(real lhs, const real3& rhs) {
    return simd::Div(Set3(lhs), rhs);
}
CUDA_HOST_DEVICE real3 operator-(const real3& a) {
    return simd::Negate(a);
}
//========================================================

CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(*, real, real3);
CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(/, real, real3);
CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(+, real, real3);
CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(-, real, real3);

CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(*, real3, real3);
CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(/, real3, real3);
CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(+, real3, real3);
CUDA_HOST_DEVICE OPERATOR_EQUALS_IMPL(-, real3, real3);

CUDA_HOST_DEVICE real Dot(const real3& v1, const real3& v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    // return simd::Dot3(v1, v2);
}
CUDA_HOST_DEVICE real Dot(const real3& v) {
    return v.x * v.x + v.y * v.y + v.z * v.z;
    // return simd::Dot3(v);
}

CUDA_HOST_DEVICE real3 Normalize(const real3& v) {
    // return simd::Normalize3(v);
    return v / Sqrt(Dot(v));
}
CUDA_HOST_DEVICE real Length(const real3& v) {
    return Sqrt(Dot(v));
    // return simd::Length3(v);
}
CUDA_HOST_DEVICE real3 Sqrt(const real3& v) {
    return simd::SquareRoot(v);
}
CUDA_HOST_DEVICE real3 Cross(const real3& b, const real3& c) {
    return simd::Cross3(b, c);
}
CUDA_HOST_DEVICE real3 Abs(const real3& v) {
    return simd::Abs(v);
}
CUDA_HOST_DEVICE real3 Sign(const real3& v) {
    return simd::Max(simd::Min(v, Set3(1)), Set3(-1));
}
CUDA_HOST_DEVICE real3 Max(const real3& a, const real3& b) {
    return simd::Max(a, b);
}

CUDA_HOST_DEVICE real3 Min(const real3& a, const real3& b) {
    return simd::Min(a, b);
}

CUDA_HOST_DEVICE real3 Max(const real3& a, const real& b) {
    return simd::Max(a, Set3(b));
}

CUDA_HOST_DEVICE real3 Min(const real3& a, const real& b) {
    return simd::Min(a, Set3(b));
}
CUDA_HOST_DEVICE real Max(const real3& a) {
    return simd::Max(a);
}
CUDA_HOST_DEVICE real Min(const real3& a) {
    return simd::Min(a);
}

CUDA_HOST_DEVICE real Length2(const real3& v1) {
    return Dot(v1);
}

CUDA_HOST_DEVICE real SafeLength(const real3& v) {
    real len_sq = Length2(v);
    if (len_sq) {
        return Sqrt(len_sq);
    } else {
        return 0.0f;
    }
}

CUDA_HOST_DEVICE real3 SafeNormalize(const real3& v, const real3& safe) {
    real len_sq = Length2(v);
    if (len_sq > real(0)) {
        return v * InvSqrt(len_sq);
    } else {
        return safe;
    }
}

CUDA_HOST_DEVICE real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max) {
    return simd::Max(clamp_min, simd::Min(a, clamp_max));
}

CUDA_HOST_DEVICE real3 Clamp(const real3& v, real max_length) {
    real3 x = v;
    real len_sq = Dot(x);
    real inv_len = InvSqrt(len_sq);

    if (len_sq > Sqr(max_length))
        x *= inv_len * max_length;

    return x;
}

CUDA_HOST_DEVICE bool operator<(const real3& a, const real3& b) {
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
CUDA_HOST_DEVICE bool operator>(const real3& a, const real3& b) {
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
CUDA_HOST_DEVICE bool operator==(const real3& a, const real3& b) {
    return (a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]);
    // return simd::IsEqual(a, b);
}
CUDA_HOST_DEVICE real3 Round(const real3& v) {
    return simd::Round(v);
}
CUDA_HOST_DEVICE bool IsZero(const real3& v) {
    return simd::IsZero(v, C_EPSILON);
}
CUDA_HOST_DEVICE real3 OrthogonalVector(const real3& v) {
    real3 abs = Abs(v);
    if (abs.x < abs.y) {
        return abs.x < abs.z ? real3(0, v.z, -v.y) : real3(v.y, -v.x, 0);
    } else {
        return abs.y < abs.z ? real3(-v.z, 0, v.x) : real3(v.y, -v.x, 0);
    }
}
CUDA_HOST_DEVICE real3 UnitOrthogonalVector(const real3& v) {
    return Normalize(OrthogonalVector(v));
}

CUDA_HOST_DEVICE void Sort(real& a, real& b, real& c) {
    if (a > b)
        Swap(a, b);
    if (b > c)
        Swap(b, c);
    if (a > b)
        Swap(a, b);
}

CUDA_HOST_DEVICE void Print(real3 v, const char* name) {
    printf("%s\n", name);
    printf("%f %f %f\n", v[0], v[1], v[2]);
}
}
#endif
