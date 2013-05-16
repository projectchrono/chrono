#ifndef CHCUDAMATH_H
#define CHCUDAMATH_H

//////////////////////////////////////////////////
//
//   ChCudaDefines.h
//
///////////////////////////////////////////////////

#include "ChCudaDefines.h"
#define R3  make_real3
#define R4  make_real4
#define R2  make_real2
#define I4  int4
#define I3  _make_int3
#define I2  _make_int2
#define U3  make_uint3
#define ZERO_EPSILON 1e-8
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
	real x, y;
};

struct real3 {
	real x, y, z;
};
struct real4 {
	real w, x, y, z;
};

static ostream &operator<<(ostream &out, real2 &a) {
	out << "[" << a.x << ", " << a.y << "]" << endl;
	return out;
}

static ostream &operator<<(ostream &out, real3 &a) {
	out << "[" << a.x << ", " << a.y << ", " << a.z << "]" << endl;
	return out;
}
static ostream &operator<<(ostream &out, real4 &a) {
	out << "[" << a.w << ", " << a.x << ", " << a.y << ", " << a.z << "]" << endl;
	return out;
}

////////defines

#define ZERO_VECTOR R3(0)
typedef real4 quaternion;

////////Convert Between Types

static __host__ __device__ real2 make_real2(real a, real b) {
	real2 t;
	t.x = a;
	t.y = b;
	return t;
}

static __host__ __device__ real2 make_real2(const real3 &rhs) {
	return make_real2(rhs.x, rhs.y);
}

static __host__ __device__ real2 make_real2(const real4 &rhs) {
	return make_real2(rhs.x, rhs.y);
}

static __host__ __device__ int3 _make_int3(real3 a) {
	int3 t;
	t.x = a.x;
	t.y = a.y;
	t.z = a.z;
	return t;
}

static __host__ __device__ real3 make_real3(real a) {
	real3 t;
	t.x = a;
	t.y = a;
	t.z = a;
	return t;
}
static __host__ __device__ real3 make_real3(real a, real b, real c) {
	real3 t;
	t.x = a;
	t.y = b;
	t.z = c;
	return t;
}

static __host__ __device__ real3 make_real3(const real4 &rhs) {
	return R3(rhs.x, rhs.y, rhs.z);
}
static __host__ __device__ real4 make_real4(real d, real a, real b, real c) {
	real4 t;
	t.w = d;
	t.x = a;
	t.y = b;
	t.z = c;
	return t;
}

static __host__ __device__ real4 make_real4(const real3 &rhs) {
	return R4(0, rhs.x, rhs.y, rhs.z);
}
////////Operator - Negate

static __host__ __device__ real3 operator -(const real3 &rhs) {
	return R3(-rhs.x, -rhs.y, -rhs.z);
}

static __host__ __device__ real4 operator -(const real4 &rhs) {
	return R4(-rhs.w, -rhs.x, -rhs.y, -rhs.z);
}

////////Operator - Plus

static __host__ __device__ real2 operator +(const real2 &rhs, const real2 &lhs) {
	return R2(rhs.x + lhs.x, rhs.y + lhs.y);
}

static __host__ __device__ real3 operator +(const real &rhs, const real3 &lhs) {
	return R3(rhs + lhs.x, rhs + lhs.y, rhs + lhs.z);
}

static __host__ __device__ real3 operator +(const real3 &rhs, const real3 &lhs) {
	return R3(rhs.x + lhs.x, rhs.y + lhs.y, rhs.z + lhs.z);
}

static __host__ __device__ real4 operator +(const real4 &rhs, const real4 &lhs) {
	return R4(rhs.w + lhs.w, rhs.x + lhs.x, rhs.y + lhs.y, rhs.z + lhs.z);
}
////////Operator - minus

static __host__ __device__ real2 operator -(const real2 &rhs, const real2 &lhs) {
	return R2(rhs.x - lhs.x, rhs.y - lhs.y);
}

static __host__ __device__ real3 operator -(const real3 &rhs, const real3 &lhs) {
	return R3(rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
}

static __host__ __device__ real4 operator -(const real4 &rhs, const real4 &lhs) {
	return R4(rhs.w - lhs.w, rhs.x - lhs.x, rhs.y - lhs.y, rhs.z - lhs.z);
}
////////Operator - times

static __host__ __device__ real2 operator *(const real2 &rhs, const real2 &lhs) {
	return R2(rhs.x * lhs.x, rhs.y * lhs.y);
}

static __host__ __device__ real3 operator *(const real3 &rhs, const real3 &lhs) {
	return R3(rhs.x * lhs.x, rhs.y * lhs.y, rhs.z * lhs.z);
}

static __host__ __device__ real4 operator *(const real4 &rhs, const real4 &lhs) {
	return R4(rhs.w * lhs.w, rhs.x * lhs.x, rhs.y * lhs.y, rhs.z * lhs.z);
}

static __host__ __device__ real2 operator *(const real2 &rhs, const real &lhs) {
	return R2(rhs.x * lhs, rhs.y * lhs);
}

static __host__ __device__ real3 operator *(const real &lhs, const real3 &rhs) {
	return R3(rhs.x * lhs, rhs.y * lhs, rhs.z * lhs);
}

static __host__ __device__ real3 operator *(const real3 &rhs, const real &lhs) {
	return R3(rhs.x * lhs, rhs.y * lhs, rhs.z * lhs);
}

static __host__ __device__ real4 operator *(const real4 &rhs, const real &lhs) {
	return R4(rhs.w * lhs, rhs.x * lhs, rhs.y * lhs, rhs.z * lhs);
}

////////Operator - divide
static __host__ __device__ real2 operator /(const real2 &rhs, const real2 &lhs) {
	return R2(rhs.x / lhs.x, rhs.y / lhs.y);
}

static __host__ __device__ real3 operator /(const real3 &rhs, const real3 &lhs) {
	return R3(rhs.x / lhs.x, rhs.y / lhs.y, rhs.z / lhs.z);
}

static __host__ __device__ real4 operator /(const real4 &rhs, const real4 &lhs) {
	return R4(rhs.w / lhs.w, rhs.x / lhs.x, rhs.y / lhs.y, rhs.z / lhs.z);
}

static __host__ __device__ real2 operator /(const real2 &rhs, const real &lhs) {
	return R2(rhs.x / lhs, rhs.y / lhs);
}

static __host__ __device__ real3 operator /(const real3 &rhs, const real &lhs) {
	return R3(rhs.x / lhs, rhs.y / lhs, rhs.z / lhs);
}

static __host__ __device__ real4 operator /(const real4 &rhs, const real &lhs) {
	return R4(rhs.w / lhs, rhs.x / lhs, rhs.y / lhs, rhs.z / lhs);
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
static __host__ __device__              inline real dot(const real3 &a, const real3 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
static __host__ __device__              inline real dot(const real4 &a, const real4 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

template<class T, class U> // dot product of the first three elements of real3/real4 values
inline __host__        __device__ real dot3(const T &a, const U &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static __host__ __device__              inline real3 cross(const real3 &a, const real3 &b) {
	real3 result;
	result.x = (a.y * b.z) - (a.z * b.y);
	result.y = (a.z * b.x) - (a.x * b.z);
	result.z = (a.x * b.y) - (a.y * b.x);
	return result;

}
template<class T>
static __host__ __device__ real length(const T &a) {
	return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}
template<class T>
static __host__ __device__ real3 normalize(const T &a) {

	real len = length(a);
	if (len < 1.e-20) {
		return R3(0, 1, 0);
	}
	return a * 1.0 / len;
}

static __host__ __device__              inline real3 ceil(real3 a) {
	return R3(ceil(a.x), ceil(a.y), ceil(a.z));
}

////////Quaternion Operations

static __host__ __device__ quaternion normalize(const quaternion &a) {
	real length = sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
	if (length < ZERO_EPSILON) {
		return R4(1, 0, 0, 0);
	}
	length = 1.0 / length;
	return R4(a.w * length, a.x * length, a.y * length, a.z * length);
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

static __host__ __device__              inline quaternion inv(quaternion a) {
	quaternion temp;
	real t1 = a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z;
	t1 = 1.0 / t1;
	temp.w = t1 * a.w;
	temp.x = -t1 * a.x;
	temp.y = -t1 * a.y;
	temp.z = -t1 * a.z;
	return temp;
}
static __host__ __device__              inline quaternion mult2(const quaternion &qa, const quaternion &qb) {
	quaternion temp;

	temp.w = qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
	temp.x = qa.w * qb.x + qa.x * qb.w - qa.z * qb.y + qa.y * qb.z;
	temp.y = qa.w * qb.y + qa.y * qb.w + qa.z * qb.x - qa.x * qb.z;
	temp.z = qa.w * qb.z + qa.z * qb.w - qa.y * qb.x + qa.x * qb.y;
	return temp;
}
static __host__ __device__              inline quaternion mult(const quaternion &a, const quaternion &b) {
	quaternion temp;

	temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	temp.x = a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y;
	temp.y = a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z;
	temp.z = a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x;
	return temp;
}

static __host__ __device__              inline real3 quatRotate(const real3 &v, const quaternion &q) {
	quaternion r = mult(mult(q, R4(0, v.x, v.y, v.z)), inv(q));
	return R3(r.x, r.y, r.z);
}
static __host__ __device__              inline real3 quatRotateMat(const real3 &v, const quaternion &q) {

	real3 result;
	result.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * v.x + (2 * q.x * q.y - 2 * q.w * q.z) * v.y + (2 * q.x * q.z + 2 * q.w * q.y) * v.z;
	result.y = (2 * q.x * q.y + 2 * q.w * q.z) * v.x + (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * v.y + (2 * q.y * q.z - 2 * q.w * q.x) * v.z;
	result.z = (2 * q.x * q.z - 2 * q.w * q.y) * v.x + (2 * q.y * q.z + 2 * q.w * q.x) * v.y + (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * v.z;
	return result;
}
static __host__ __device__              inline real3 quatRotateMatT(const real3 &v, const quaternion &q) {

	real3 result;
	result.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * v.x + (2 * q.x * q.y + 2 * q.w * q.z) * v.y + (2 * q.x * q.z - 2 * q.w * q.y) * v.z;
	result.y = (2 * q.x * q.y - 2 * q.w * q.z) * v.x + (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * v.y + (2 * q.y * q.z + 2 * q.w * q.x) * v.z;
	result.z = (2 * q.x * q.z + 2 * q.w * q.y) * v.x + (2 * q.y * q.z - 2 * q.w * q.x) * v.y + (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * v.z;
	return result;
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
	if (fabs(ab) < ZERO_EPSILON)
		return 1;
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
	} else if (x > 0) {
		return 1;
	} else {
		return 0;
	}
}

////////Generic Operations
template<class T>
static __host__ __device__ inline void Swap(T &a, T &b) {
	T tmp = a;
	a = b;
	b = tmp;
}

static __host__ __device__ real3 fabs(const real3 &a) {
	return R3(fabs(a.x), fabs(a.y), fabs(a.z));
}

//template<class T>
//static __host__ __device__ T fabs(const T &a) {
//    return T(fabs(a.x), fabs(a.y), fabs(a.z));
//}

template<class T>
__host__ __device__ inline real max3(T a) {
	return max(a.x, max(a.y, a.z));
}
template<class T>
__host__ __device__ inline real min3(T a) {
	return min(a.x, min(a.y, a.z));
}

////////Output Operations
template<class T>
static ostream &operator<<(ostream &out, const thrust::device_vector<T> &x) {
	for (uint i = 0; i < x.size(); i++) {
		out << x[i] << endl;
	}
	return out;
}
template<class T>
static ostream &operator<<(ostream &out, const thrust::host_vector<T> &x) {
	for (uint i = 0; i < x.size(); i++) {
		out << x[i] << endl;
	}
	return out;
}

struct M33 {
	real3 U, V, W;
	//[U.x,V.x,W.x]
	//[U.y,V.y,W.y]
	//[U.z,V.z,W.z]
	//transposed:
	//[U.x,U.y,U.z]
	//[V.x,V.y,V.z]
	//[W.x,W.y,W.z]

	//[U.x,V.x,W.x][x]
	//[U.y,V.y,W.y][y]
	//[U.z,V.z,W.z][z]
};

static __host__ __device__ M33 XMatrix(real3 vect) {
	M33 Xmat;
	Xmat.U.x = 0;
	Xmat.V.x = -vect.z;
	Xmat.W.x = vect.y;

	Xmat.U.y = vect.z;
	Xmat.V.y = 0;
	Xmat.W.y = -vect.x;

	Xmat.U.z = -vect.y;
	Xmat.V.z = vect.x;
	Xmat.W.z = 0;
	return Xmat;
}
static __host__ __device__ M33 MatMult(M33 A, M33 B) {
	M33 result;
	result.U.x = A.U.x * B.U.x + A.V.x * B.U.y + A.W.x * B.U.z; //row1 * col1
	result.V.x = A.U.x * B.V.x + A.V.x * B.V.y + A.W.x * B.V.z; //row1 * col2
	result.W.x = A.U.x * B.W.x + A.V.x * B.W.y + A.W.x * B.W.z; //row1 * col3

	result.U.y = A.U.y * B.U.x + A.V.y * B.U.y + A.W.y * B.U.z; //row2 * col1
	result.V.y = A.U.y * B.V.x + A.V.y * B.V.y + A.W.y * B.V.z; //row2 * col2
	result.W.y = A.U.y * B.W.x + A.V.y * B.W.y + A.W.y * B.W.z; //row2 * col3

	result.U.z = A.U.z * B.U.x + A.V.z * B.U.y + A.W.z * B.U.z; //row3 * col1
	result.V.z = A.U.z * B.V.x + A.V.z * B.V.y + A.W.z * B.V.z; //row3 * col2
	result.W.z = A.U.z * B.W.x + A.V.z * B.W.y + A.W.z * B.W.z; //row3 * col3

	return result;
}
static __host__ __device__ real3 MatMult(M33 A, real3 B) {
	real3 result;

	result.x = A.U.x * B.x + A.V.x * B.y + A.W.x * B.z; //row1 * col1
	result.y = A.U.y * B.x + A.V.y * B.y + A.W.y * B.z; //row2 * col2
	result.z = A.U.z * B.x + A.V.z * B.y + A.W.z * B.z; //row3 * col3

	return result;
}

//A is transposed
static __host__ __device__ M33 MatTMult(M33 A, M33 B) {
	M33 result;
	result.U.x = A.U.x * B.U.x + A.U.y * B.U.y + A.U.z * B.U.z; //row1 * col1
	result.V.x = A.U.x * B.V.x + A.U.y * B.V.y + A.U.z * B.V.z; //row1 * col2
	result.W.x = A.U.x * B.W.x + A.U.y * B.W.y + A.U.z * B.W.z; //row1 * col3

	result.U.y = A.V.x * B.U.x + A.V.y * B.U.y + A.V.z * B.U.z; //row2 * col1
	result.V.y = A.V.x * B.V.x + A.V.y * B.V.y + A.V.z * B.V.z; //row2 * col2
	result.W.y = A.V.x * B.W.x + A.V.y * B.W.y + A.V.z * B.W.z; //row2 * col3

	result.U.z = A.W.x * B.U.x + A.W.y * B.U.y + A.W.z * B.U.z; //row3 * col1
	result.V.z = A.W.x * B.V.x + A.W.y * B.V.y + A.W.z * B.V.z; //row3 * col2
	result.W.z = A.W.x * B.W.x + A.W.y * B.W.y + A.W.z * B.W.z; //row3 * col3

	return result;
}

static __host__ __device__ real3 MatTMult(M33 A, real3 B) {
	real3 result;

	result.x = A.U.x * B.x + A.U.y * B.y + A.U.z * B.z; //row1 * col1
	result.y = A.V.x * B.x + A.V.y * B.y + A.V.z * B.z; //row2 * col2
	result.z = A.W.x * B.x + A.W.y * B.y + A.W.z * B.z; //row3 * col3

	return result;
}

static __host__ __device__ M33 AMat(real4 q) {
	M33 result;

	real e0e0 = q.w * q.w;
	real e1e1 = q.x * q.x;
	real e2e2 = q.y * q.y;
	real e3e3 = q.z * q.z;
	real e0e1 = q.w * q.x;
	real e0e2 = q.w * q.y;
	real e0e3 = q.w * q.z;
	real e1e2 = q.x * q.y;
	real e1e3 = q.x * q.z;
	real e2e3 = q.y * q.z;
	result.U.x = (e0e0 + e1e1) * 2 - 1;
	result.V.x = (e1e2 - e0e3) * 2;
	result.W.x = (e1e3 + e0e2) * 2;

	result.U.y = (e1e2 + e0e3) * 2;
	result.V.y = (e0e0 + e2e2) * 2 - 1;
	result.W.y = (e2e3 - e0e1) * 2;

	result.U.z = (e1e3 - e0e2) * 2;
	result.V.z = (e2e3 + e0e1) * 2;
	result.W.z = (e0e0 + e3e3) * 2 - 1;

//	result.U.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);
//	result.V.x = (2 * q.x * q.y - 2 * q.w * q.z);
//	result.W.x = (2 * q.x * q.z + 2 * q.w * q.y);
//
//	result.U.y = (2 * q.x * q.y + 2 * q.w * q.z);
//	result.V.y = (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z);
//	result.W.y = (2 * q.y * q.z - 2 * q.w * q.x);
//
//	result.U.z = (2 * q.x * q.z - 2 * q.w * q.y);
//	result.V.z = (2 * q.y * q.z + 2 * q.w * q.x);
//	result.W.z = (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
	return result;

}
//[U.x,U.y,U.z]
//[V.x,V.y,V.z]
//[W.x,W.y,W.z]
static __host__ __device__ M33 AMatT(real4 q) {
	M33 result;

	real e0e0 = q.w * q.w;
	real e1e1 = q.x * q.x;
	real e2e2 = q.y * q.y;
	real e3e3 = q.z * q.z;
	real e0e1 = q.w * q.x;
	real e0e2 = q.w * q.y;
	real e0e3 = q.w * q.z;
	real e1e2 = q.x * q.y;
	real e1e3 = q.x * q.z;
	real e2e3 = q.y * q.z;
	result.U.x = (e0e0 + e1e1) * 2 - 1;
	result.U.y = (e1e2 - e0e3) * 2;
	result.U.z = (e1e3 + e0e2) * 2;

	result.V.x = (e1e2 + e0e3) * 2;
	result.V.y = (e0e0 + e2e2) * 2 - 1;
	result.V.z = (e2e3 - e0e1) * 2;

	result.W.x = (e1e3 - e0e2) * 2;
	result.W.y = (e2e3 + e0e1) * 2;
	result.W.z = (e0e0 + e3e3) * 2 - 1;

	return result;

}
static __host__ __device__ M33 AbsMat(M33 A) {
	M33 result;
	result.U.x = fabs(A.U.x);
	result.U.y = fabs(A.U.y);
	result.U.z = fabs(A.U.z);
	result.V.x = fabs(A.V.x);
	result.V.y = fabs(A.V.y);
	result.V.z = fabs(A.V.z);
	result.W.x = fabs(A.W.x);
	result.W.y = fabs(A.W.y);
	result.W.z = fabs(A.W.z);

	return result;

}
#endif

