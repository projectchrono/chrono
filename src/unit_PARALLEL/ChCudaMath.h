#ifndef CHCUDAMATH_H
#define CHCUDAMATH_H

//////////////////////////////////////////////////
//
//   ChCudaDefines.h
//
///////////////////////////////////////////////////

#include "ChCudaDefines.h"


#define R3  real3
#define R4  real4
#define R2  real2
#define I4  int4
#define I3  _make_int3
#define I2  _make_int2
#define U3  make_uint3
#define ZERO_EPSILON 1e-10
typedef unsigned int uint;

static __host__ __device__ int3 _make_int3(int a, int b, int c) {
    int3 t;
    t.x = a;
    t.y = b;
    t.z = c;
    return t;
}


static __host__ __device__ int2 _make_int2(int a, int b) {
    int2 t;
    t.x = a;
    t.y = b;
    return t;
}

////////Define Real, either float or double
typedef double real;
////////Structures
struct real2 {
    __host__ __device__ real2() {}
    __host__ __device__ real2(real a): x(a), y(a) {}
    __host__ __device__ real2(real a, real b): x(a), y(b) {}
    real x, y;
};

struct real3 {

    __host__ __device__ real3() {}
    __host__ __device__ real3(real a): x(a), y(a), z(a) {}
    __host__ __device__ real3(real a , real b , real c): x(a), y(b), z(c) {}

    real x, y, z;
};
struct real4 {
    __host__ __device__ real4() {}
    __host__ __device__ real4(real a): w(a), x(a), y(a), z(a) {}
    __host__ __device__ real4(real d, real a, real b, real c): w(d), x(a), y(b), z(c) {}

    real w, x, y, z;
};

////////defines

#define ZERO_VECTOR real3(0)
typedef real4 quaternion;

////////Convert Between Types

static __host__ __device__ real2 make_real2(const real3 &rhs) {
    return real2(rhs.x, rhs.y);
}

static __host__ __device__ real2 make_real2(const real4 &rhs) {
    return real2(rhs.x, rhs.y);
}

static __host__ __device__ real3 make_real3(const real4 &rhs) {
    return real3(rhs.x, rhs.y, rhs.z);
}

static __host__ __device__ real4 make_real4(const real3 &rhs) {
    return real4(0, rhs.x, rhs.y, rhs.z);
}
////////Operator - Negate

static __host__ __device__ real3 operator -(const real3 &rhs) {
    return real3(-rhs.x, -rhs.y, -rhs.z);
}

static __host__ __device__ real4 operator -(const real4 &rhs) {
    return real4(-rhs.w, -rhs.x, -rhs.y, -rhs.z);
}

////////Operator - Plus

static __host__ __device__ real2 operator +(const real2 &rhs, const real2 &lhs) {
    return real2(rhs.x + lhs.x, rhs.y + lhs.y);
}

static __host__ __device__ real3 operator +(const real3 &rhs, const real3 &lhs) {
    return real3(rhs.x + lhs.x, rhs.y + lhs.y, rhs.z + lhs.z);
}

static __host__ __device__ real4 operator +(const real4 &rhs, const real4 &lhs) {
    return real4(rhs.w + lhs.w, rhs.x + lhs.x, rhs.y + lhs.y, rhs.z + lhs.z);
}
////////Operator - minus

static __host__ __device__ real2 operator -(const real2 &rhs, const real2 &lhs) {
    return real2(rhs.x - lhs.x, rhs.y - lhs.y);
}

static __host__ __device__ real3 operator -(const real3 &rhs, const real3 &lhs) {
    return real3(rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
}

static __host__ __device__ real4 operator -(const real4 &rhs, const real4 &lhs) {
    return real4(rhs.w - lhs.w, rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
}
////////Operator - times

static __host__ __device__ real2 operator *(const real2 &rhs, const real2 &lhs) {
    return real2(rhs.x * lhs.x, rhs.y * lhs.y);
}

static __host__ __device__ real3 operator *(const real3 &rhs, const real3 &lhs) {
    return real3(rhs.x * lhs.x, rhs.y * lhs.y, rhs.z * lhs.z);
}

static __host__ __device__ real4 operator *(const real4 &rhs, const real4 &lhs) {
    return real4(rhs.w * lhs.w, rhs.x * lhs.x, rhs.y * lhs.y, rhs.z * lhs.z);
}



static __host__ __device__ real2 operator *(const real2 &rhs, const real &lhs) {
    return real2(rhs.x * lhs, rhs.y * lhs);
}

static __host__ __device__ real3 operator *(const real3 &rhs, const real &lhs) {
    return real3(rhs.x * lhs, rhs.y * lhs, rhs.z * lhs);
}

static __host__ __device__ real4 operator *(const real4 &rhs, const real &lhs) {
    return real4(rhs.w * lhs, rhs.x * lhs, rhs.y * lhs, rhs.z * lhs);
}

////////Operator - divide
static __host__ __device__ real2 operator /(const real2 &rhs, const real2 &lhs) {
    return real2(rhs.x / lhs.x, rhs.y / lhs.y);
}

static __host__ __device__ real3 operator /(const real3 &rhs, const real3 &lhs) {
    return real3(rhs.x / lhs.x, rhs.y / lhs.y, rhs.z / lhs.z);
}

static __host__ __device__ real4 operator /(const real4 &rhs, const real4 &lhs) {
    return real4(rhs.w / lhs.w, rhs.x / lhs.x, rhs.y / lhs.y, rhs.z / lhs.z);
}



static __host__ __device__ real2 operator /(const real2 &rhs, const real &lhs) {
    return real2(rhs.x / lhs, rhs.y / lhs);
}

static __host__ __device__ real3 operator /(const real3 &rhs, const real &lhs) {
    return real3(rhs.x / lhs, rhs.y / lhs, rhs.z / lhs);
}

static __host__ __device__ real4 operator /(const real4 &rhs, const real &lhs) {
    return real4(rhs.w / lhs, rhs.x / lhs, rhs.y / lhs, rhs.z / lhs);
}


////////Operator - Plus Equal, Minus Equal, Times Equal, Divided by equal

template<class T>
static __host__ __device__ void operator +=(T &rhs, const T &lhs) {
    rhs = rhs + lhs;
}
template<class T>
static __host__ __device__ void operator -=(T &rhs, const T &lhs) {
    rhs = rhs - lhs;
}
template<class T>
static __host__ __device__ void operator *=(T &rhs, const T &lhs) {
    rhs = rhs * lhs;
}

template<class T>
static __host__ __device__ void operator /=(T &rhs, const T &lhs) {
    rhs = rhs / lhs;
}
////////Operator is equal

static __host__ __device__ bool operator ==(const real2 &a, const real2 &b) {
    return ((a.x == b.x) && (a.y == b.y));
}

static __host__ __device__ bool operator ==(const real3 &a, const real3 &b) {
    return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}
static __host__ __device__ bool operator ==(const real4 &a, const real4 &b) {
    return ((a.w == b.w) && (a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

////////Vector Operations
static __host__ __device__ inline real dot(const real3 &a, const real3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static __host__ __device__ inline real dot(const real4 &a, const real4 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

template<class T, class U> // dot product of the first three elements of real3/real4 values
inline __host__ __device__ real dot3(const T &a, const U &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static __host__ __device__ inline real3 cross(const real3 &a, const real3 &b) {
    return real3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
template<class T>
static __host__ __device__ real length(const T &a) {
    return sqrt(dot(a, a));
}
template<class T>
static __host__ __device__ real3 normalize(const T &a) {
    return a / sqrt(dot(a, a));
}

static __host__ __device__ inline real3 ceil(real3 a) {
    return real3(ceil(a.x), ceil(a.y), ceil(a.z));
}

////////Quaternion Operations

static __host__ __device__ quaternion normalize(const quaternion &a) {
    real length = 1.0 / sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    return quaternion(a.w * length, a.x * length, a.y * length, a.z * length);
}

static __host__ __device__ quaternion Q_from_AngAxis(real angle, real3 axis) {
    quaternion quat;
    real halfang;
    real sinhalf;
    halfang = (angle * 0.5);
    sinhalf = sin(halfang);
    quat.w = cos(halfang);
    quat.x = axis.x * sinhalf;
    quat.y = axis.y * sinhalf;
    quat.z = axis.z * sinhalf;
    return (quat);
}

static __host__ __device__ inline quaternion inv(quaternion a) {
    quaternion temp;
    real t1 = a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z;
    t1 = 1.0 / t1;
    temp.w = t1 * a.w;
    temp.x = -t1 * a.x;
    temp.y = -t1 * a.y;
    temp.z = -t1 * a.z;
    return temp;
}

static __host__ __device__ inline quaternion mult(const quaternion &a, const quaternion &b) {
    quaternion temp;
    temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    temp.x = a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y;
    temp.y = a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z;
    temp.z = a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x;
    return temp;
}

static __host__ __device__ inline real3 quatRotate(const real3 &v, const quaternion &q) {
    quaternion r = mult(mult(q, quaternion(0, v.x, v.y, v.z)), inv(q));
    return real3(r.x, r.y, r.z);
}

static __host__ __device__ quaternion operator %(const quaternion rhs, const quaternion lhs) {
    return mult(rhs, lhs);
}

////////Check if zero or equal within tolerance
static __host__ __device__ bool IsZero(const real &a) {
    return fabs(a) < ZERO_EPSILON;
}

static __host__ __device__ bool IsZero(const real3 &a) {
    return IsZero(a.x) && IsZero(a.y) && IsZero(a.z);
}

static __host__ __device__ bool isEqual(const real &_a, const real &_b) {
    real ab;
    ab = fabs(_a - _b);

    if (fabs(ab) < ZERO_EPSILON) return 1;

    real a, b;
    a = fabs(_a);
    b = fabs(_b);

    if (b > a) {
        return ab < ZERO_EPSILON * b;
    } else {
        return ab < ZERO_EPSILON * a;
    }
}

static __host__ __device__ bool isEqual(const real3 &a, const real3 &b) {
    return isEqual(a.x, b.x) && isEqual(a.y, b.y) && isEqual(a.z, b.z);
}

////////Other Operations
static __host__ __device__ uint nearest_pow(uint num) {
    uint n = num > 0 ? num - 1 : 0;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
}

static __host__ __device__ real sign(real x) {
    if (x < 0) {
        return -1;
    } else {
        return 1;
    }
}

////////Generic Operations
template<class T>
static __host__ __device__ inline void Swap(T &a, T &b) {
    T tmp = a;
    a = b;
    b = tmp;
}
template<class T>
static __host__ __device__ T fabs(const T &a) {
    return T(fabs(a.x), fabs(a.y), fabs(a.z));
}

template<class T>
__host__ __device__ inline real max3(T a) {
    return max(a.x, max(a.y, a.z));
}
template<class T>
__host__ __device__ inline real min3(T a) {
    return min(a.x, min(a.y, a.z));
}



#endif

