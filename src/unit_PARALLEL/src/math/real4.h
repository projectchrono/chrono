#ifndef REAL4_H
#define REAL4_H

#include "ChParallelDefines.h"
#include "real2.h"
#include "real3.h"

#define R4  real4

class __attribute__ ((aligned(16))) real4 {
public:
	union {
		struct {
			real w, x, y, z;
		};
		__m128 mmvalue;
	};

	inline real4() : mmvalue(_mm_setzero_ps()) {}
	inline real4(float a) : mmvalue(_mm_set1_ps(a)) {}
	inline real4(float a, float b, float c) :  mmvalue(_mm_setr_ps(0, a,b,c)) {}
	inline real4(float d, float a, float b, float c) :  mmvalue(_mm_setr_ps(d, a,b,c)) {}
	inline real4(__m128 m) : mmvalue(m) {}

	inline real4 operator+(const real4& b) const { return _mm_add_ps(mmvalue, b.mmvalue);}
	inline real4 operator-(const real4& b) const { return _mm_sub_ps(mmvalue, b.mmvalue);}
	inline real4 operator*(const real4& b) const { return _mm_mul_ps(mmvalue, b.mmvalue);}
	inline real4 operator/(const real4& b) const { return _mm_div_ps(mmvalue, b.mmvalue);}
	inline real4 operator-()               const { return _mm_xor_ps(mmvalue, SIGNMASK); }

	inline real4& operator+=(const real4& b) { *this = *this + b; return *this; }
	inline real4& operator-=(const real4& b) { *this = *this - b; return *this; }
	inline real4& operator*=(const real4& b) { *this = *this * b; return *this; }
	inline real4& operator/=(const real4& b) { *this = *this / b; return *this; }

	inline real4 operator+(real b) const { return _mm_add_ps(mmvalue, _mm_set1_ps(b)); }
	inline real4 operator-(real b) const { return _mm_sub_ps(mmvalue, _mm_set1_ps(b)); }
	inline real4 operator*(real b) const { return _mm_mul_ps(mmvalue, _mm_set1_ps(b)); }
	inline real4 operator/(real b) const { return _mm_div_ps(mmvalue, _mm_set1_ps(b)); }

	inline real4& operator+=(real b) { *this = *this + b; return *this; }
	inline real4& operator-=(real b) { *this = *this - b; return *this; }
	inline real4& operator*=(real b) { *this = *this * b; return *this; }
	inline real4& operator/=(real b) { *this = *this / b; return *this; }

};

inline real4 operator+(real a, const real4& b) { return b + a; }
inline real4 operator-(real a, const real4& b) { return real4(a) - b; }
inline real4 operator*(real a, const real4& b) { return b * a; }
inline real4 operator/(real a, const real4& b) { return real4(a) / b; }



typedef real4 quaternion;

static inline ostream &operator<<(ostream &out, real4 &a) {
	out << "[" << a.w << ", " << a.x << ", " << a.y << ", " << a.z << "]" << endl;
	return out;
}

static inline real3 make_real3(const real4 &rhs) {
	return R3(rhs.x, rhs.y, rhs.z);
}


static inline bool operator ==(const real4 &a, const real4 &b) {
	return ((a.w == b.w) && (a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}

static inline real dot(const real4 &a, const real4 &b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline quaternion normalize(const quaternion &a) {
	real length = sqrt(a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z);
	if (length < ZERO_EPSILON) {
		return R4(1, 0, 0, 0);
	}
	length = 1.0 / length;
	return R4(a.w * length, a.x * length, a.y * length, a.z * length);
}

static inline quaternion Q_from_AngAxis(const real &angle, const real3 &axis) {
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

static inline quaternion inv(const quaternion &a) {
	quaternion temp;
	real t1 = a.w * a.w + a.x * a.x + a.y * a.y + a.z * a.z;
	t1 = 1.0 / t1;
	temp.w = t1 * a.w;
	temp.x = -t1 * a.x;
	temp.y = -t1 * a.y;
	temp.z = -t1 * a.z;
	return temp;
}
static inline quaternion mult2(const quaternion &qa, const quaternion &qb) {
	quaternion temp;

	temp.w = qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
	temp.x = qa.w * qb.x + qa.x * qb.w - qa.z * qb.y + qa.y * qb.z;
	temp.y = qa.w * qb.y + qa.y * qb.w + qa.z * qb.x - qa.x * qb.z;
	temp.z = qa.w * qb.z + qa.z * qb.w - qa.y * qb.x + qa.x * qb.y;
	return temp;
}
static inline quaternion mult(const quaternion &a, const quaternion &b) {
	quaternion temp;

	temp.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
	temp.x = a.w * b.x + b.w * a.x + a.y * b.z - a.z * b.y;
	temp.y = a.w * b.y + b.w * a.y + a.z * b.x - a.x * b.z;
	temp.z = a.w * b.z + b.w * a.z + a.x * b.y - a.y * b.x;
	return temp;
}

static inline quaternion lerp(const quaternion &a, const quaternion &b, real alpha) {
	return normalize(a + alpha * (b - a));

}

static inline real angle(const quaternion &a, const quaternion &b) {

	real s = sqrtf(dot(a, a) * dot(b, b));

	return acos(dot(a, b) / s);

}
static inline quaternion slerp(const quaternion &a, const quaternion &b, real alpha) {
	quaternion ans = a;
	real theta = angle(a, b);
	if (theta != 0) {
		real d = 1.0 / sin(theta);
		real s0 = sin((1.0 - alpha) * theta);
		real s1 = sin(alpha * theta);
		if (dot(a, b) < 0) {
			ans.x = (a.x * s0 - b.x * s1) * d;
			ans.y = (a.y * s0 - b.y * s1) * d;
			ans.z = (a.z * s0 - b.z * s1) * d;
			ans.w = (a.w * s0 - b.w * s1) * d;

		} else {
			ans.x = (a.x * s0 + b.x * s1) * d;
			ans.y = (a.y * s0 + b.y * s1) * d;
			ans.z = (a.z * s0 + b.z * s1) * d;
			ans.w = (a.w * s0 + b.w * s1) * d;

		}

	}
	return ans;

}

static inline real3 quatRotate(const real3 &v, const quaternion &q) {
	quaternion r = mult(mult(q, R4(0, v.x, v.y, v.z)), inv(q));
	return R3(r.x, r.y, r.z);
}
static inline real3 quatRotateMat(const real3 &v, const quaternion &q) {

	real3 result;
	result.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * v.x + (2 * q.x * q.y - 2 * q.w * q.z) * v.y
			+ (2 * q.x * q.z + 2 * q.w * q.y) * v.z;
	result.y = (2 * q.x * q.y + 2 * q.w * q.z) * v.x + (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * v.y
			+ (2 * q.y * q.z - 2 * q.w * q.x) * v.z;
	result.z = (2 * q.x * q.z - 2 * q.w * q.y) * v.x + (2 * q.y * q.z + 2 * q.w * q.x) * v.y
			+ (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * v.z;
	return result;
}
static inline real3 quatRotateMatT(const real3 &v, const quaternion &q) {

	real3 result;
	result.x = (q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z) * v.x + (2 * q.x * q.y + 2 * q.w * q.z) * v.y
			+ (2 * q.x * q.z - 2 * q.w * q.y) * v.z;
	result.y = (2 * q.x * q.y - 2 * q.w * q.z) * v.x + (q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z) * v.y
			+ (2 * q.y * q.z + 2 * q.w * q.x) * v.z;
	result.z = (2 * q.x * q.z + 2 * q.w * q.y) * v.x + (2 * q.y * q.z - 2 * q.w * q.x) * v.y
			+ (q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z) * v.z;
	return result;
}
static inline quaternion operator %(const quaternion rhs, const quaternion lhs) {
	return mult(rhs, lhs);
}

#endif
