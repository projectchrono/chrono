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
#define I3  make_int3
#define I2  make_int2
#define U3  make_uint3
typedef unsigned int uint;



typedef double real;
struct real2;
struct real3;
struct real4;

struct real2 {
    __host__ __device__ real2(real a = 0, real b = 0): x(a), y(b) {}
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
    __host__ __device__ real4(real3 a): x(a.x), y(a.y), z(a.z), w(0) {}
    __host__ __device__ real4(real d , real a , real b , real c ): w(d), x(a), y(b), z(c) {}

    real w, x, y, z;
};

static __device__ __host__ real3 make_real3(const real4 &rhs) {
    return real3(rhs.x,rhs.y,rhs.z);
}

static __device__ __host__ real3 operator +(const real3 &rhs, const real3 &lhs) {
    real3 temp;
    temp.x = rhs.x + lhs.x;
    temp.y = rhs.y + lhs.y;
    temp.z = rhs.z + lhs.z;
    return temp;
}
static __device__ __host__ real4 operator +(const real4 &rhs, const real4 &lhs) {
    real4 temp;
    temp.w = rhs.w + lhs.w;
    temp.x = rhs.x + lhs.x;
    temp.y = rhs.y + lhs.y;
    temp.z = rhs.z + lhs.z;

    return temp;
}
static __device__ __host__ real3 operator -(const real3 &rhs) {
    return real3(-rhs.x, -rhs.y, -rhs.z);
}

static __device__ __host__ real3 operator -(const real3 &rhs, const real3 &lhs) {
    real3 temp;
    temp.x = rhs.x - lhs.x;
    temp.y = rhs.y - lhs.y;
    temp.z = rhs.z - lhs.z;
    return temp;
}
static __device__ __host__ void operator +=(real3 &rhs, const real3 &lhs) {
    rhs = rhs + lhs;
}

static __device__ __host__ void operator -=(real3 &rhs, const real3 &lhs) {
    rhs = rhs - lhs;
}

static __device__ __host__ real3 operator *(const real3 &rhs, const real3 &lhs) {
    real3 temp;
    temp.x = rhs.x * lhs.x;
    temp.y = rhs.y * lhs.y;
    temp.z = rhs.z * lhs.z;
    return temp;
}

static __device__ __host__ real3 operator *(const real4 &rhs, const real3 &lhs) {
    real3 temp;
    temp.x = rhs.x * lhs.x;
    temp.y = rhs.y * lhs.y;
    temp.z = rhs.z * lhs.z;
    return temp;
}

static __device__ __host__ real3 operator *(const real3 &rhs, const real &lhs) {
    real3 temp;
    temp.x = rhs.x * lhs;
    temp.y = rhs.y * lhs;
    temp.z = rhs.z * lhs;
    return temp;
}
static __device__ __host__ real4 operator *(const real4 &rhs, const real &lhs) {
    real4 temp;
    temp.w = rhs.w * lhs;
    temp.x = rhs.x * lhs;
    temp.y = rhs.y * lhs;
    temp.z = rhs.z * lhs;
    return temp;
}

static __device__ __host__ real3 operator /(const real3 &rhs, const real3 &lhs) {
    real3 temp;
    temp.x = rhs.x / lhs.x;
    temp.y = rhs.y / lhs.y;
    temp.z = rhs.z / lhs.z;
    return temp;
}
static __device__ __host__ real3 operator /(const real3 &rhs, const real &lhs) {
    real3 temp;
    temp.x = rhs.x / lhs;
    temp.y = rhs.y / lhs;
    temp.z = rhs.z / lhs;
    return temp;
}

static __device__ __host__ real4 operator /(const real4 &rhs, const real &lhs) {
    real4 temp;
    temp.w = rhs.w / lhs;
    temp.x = rhs.x / lhs;
    temp.y = rhs.y / lhs;
    temp.z = rhs.z / lhs;
    return temp;
}




#define Zero_Vector real3(0)


static __device__ __host__ inline real dot(const real3 &a, const real3 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
static __device__ __host__ inline real dot(const real4 &a, const real4 &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z+ a.w * b.w;
}

template<class T, class U> // dot product of the first three elements of real3/real4 values
inline __host__ __device__ real dot3(const T &a, const U &b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static __device__ __host__ inline real3 cross(real3 a, real3 b) {
    return real3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static __device__ __host__ real length(const real3 &a) {
    return sqrt(dot(a, a));
}

static __device__ __host__ real3 normalize(const real3 &a) {
    return a / real(sqrt(dot(a, a)));
}

static __device__ __host__ real4 Q_from_AngAxis(real angle, real3 axis) {
    real4 quat;
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

static __device__ __host__ real4 normalize(const real4 &a) {
    real length = 1.0 / sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
    return real4(a.w * length, a.x * length, a.y * length, a.z * length);
}

static __device__ __host__ inline real4 inv(real4 a) {
    real4 temp;
    real t1 = a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z;
    t1 = 1.0 / t1;
    temp.w = t1 * a.w;
    temp.x = -t1 * a.x;
    temp.y = -t1 * a.y;
    temp.z = -t1 * a.z;
    return temp;
}

static __device__ __host__ inline real4 mult(const real4 &a, const real4 &b) {
    real4 temp;
    temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    temp.x = a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y;
    temp.y = a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z;
    temp.z = a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x;
    return temp;
}

static __device__ __host__ inline real3 quatRotate(const real3 &v, const real4 &q) {
    real4 r = mult(mult(q, real4(0, v.x, v.y, v.z)), inv(q));
    return real3(r.x, r.y, r.z);
}

static __device__ __host__ real4 operator %(const real4 rhs, const real4 lhs) {
    return mult(rhs, lhs);
}

//-----------------------------------------------------------------
//template<class T>
static __device__ __host__ bool operator ==(const real3 &a, const real3 &b) {
    return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}
template<class T>
static __device__ __host__ T fabs(const T &a) {
    return T(fabs(a.x), fabs(a.y), fabs(a.z));
}


static __device__ __host__ uint nearest_pow(uint num) {
    uint n = num > 0 ? num - 1 : 0;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
}

__host__ __device__ inline real3 ceil(real3 v) {
    return real3(ceil(v.x), ceil(v.y), ceil(v.z));
}
template<class T>
__host__ __device__ inline real max3(T a) {
    return max(a.x, max(a.y, a.z));
}
template<class T>
__host__ __device__ inline real min3(T a) {
    return min(a.x, min(a.y, a.z));
}

static __device__ __host__ bool IsZero3(const real3 &v) {
    return (v.x < Vector_ZERO_EPSILON && v.x > -Vector_ZERO_EPSILON && v.y < Vector_ZERO_EPSILON && v.y > -Vector_ZERO_EPSILON && v.z < Vector_ZERO_EPSILON && v.z > -Vector_ZERO_EPSILON);
}
static __device__ __host__ bool IsZero(const real &val) {
    return fabs(val) < 1E-10;
}
static __device__ __host__ bool isEqual(const real &_a, const real &_b) {
    real ab;
    ab = fabs(_a - _b);

    if (fabs(ab) < 1E-10) return 1;

    real a, b;
    a = fabs(_a);
    b = fabs(_b);

    if (b > a) {
        return ab < 1E-10 * b;
    } else {
        return ab < 1E-10 * a;
    }
}

template<class T>
static __device__ __host__ inline void Swap(T &a, T &b) {
    T tmp = a;
    a = b;
    b = tmp;
}

// collision detection structures
struct AABB {
    real3 min, max;
};


typedef thrust::pair<real3, real3> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction: public thrust::binary_function<bbox, bbox, bbox> {
    bbox __host__ __device__ operator()(
        bbox a,
        bbox b) {
        real3 ll = R3(fmin(a.first.x, b.first.x), fmin(a.first.y, b.first.y), fmin(a.first.z, b.first.z)); // lower left corner
        real3 ur = R3(fmax(a.second.x, b.second.x), fmax(a.second.y, b.second.y), fmax(a.second.z, b.second.z)); // upper right corner
        return bbox(ll, ur);
    }
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation: public thrust::unary_function<real3, bbox> {
    bbox __host__ __device__ operator()(
        real3 point) {
        return bbox(point, point);
    }
};

#endif
