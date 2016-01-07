#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/simd.h"
#include "chrono_parallel/math/real3.h"

namespace chrono {

//========================================================
real3 operator+(const real3& a, const real3& b) {
	return simd::Add(a, b);
}
real3 operator-(const real3& a, const real3& b) {
	return simd::Sub(a, b);
}
real3 operator*(const real3& a, const real3& b) {
	return simd::Mul(a, b);
}
real3 operator/(const real3& a, const real3& b) {
	return simd::Div(a, b);
}
//========================================================
real3 operator+(const real3& a, real b) {
	return simd::Add(a, real3::Set(b));
}
real3 operator-(const real3& a, real b) {
	return simd::Sub(a, real3::Set(b));
}
real3 operator*(const real3& a, real b) {
	return simd::Mul(a, real3::Set(b));
}
real3 operator/(const real3& a, real b) {
	return simd::Div(a, real3::Set(b));
}
real3 operator*(real lhs, const real3& rhs) {
	return simd::Mul(real3::Set(lhs), rhs);
}
real3 operator/(real lhs, const real3& rhs) {
	return simd::Div(real3::Set(lhs), rhs);
}
real3 operator-(const real3& a) {
	return simd::Negate(a);
}
//========================================================

real Dot(const real3& v1, const real3& v2) {
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	// return simd::Dot3(v1, v2);
}
real Dot(const real3& v) {
	return v.x * v.x + v.y * v.y + v.z * v.z;
	// return simd::Dot3(v);
}

real3 Normalize(const real3& v) {
	// return simd::Normalize3(v);
	return v / Sqrt(Dot(v));
}
real Length(const real3& v) {
	return Sqrt(Dot(v));
	// return simd::Length3(v);
}
real3 Sqrt(const real3& v) {
	return simd::SquareRoot(v);
}
real3 Cross(const real3& b, const real3& c) {
	return simd::Cross3(b.array, c.array);
}
real3 Abs(const real3& v) {
	return simd::Abs(v);
}
real3 Sign(const real3& v) {
	return simd::Max(simd::Min(v, real3::Set(1)), real3::Set(-1));
}
real3 Max(const real3& a, const real3& b) {
	return simd::Max(a, b);
}

real3 Min(const real3& a, const real3& b) {
	return simd::Min(a, b);
}

real3 Max(const real3& a, const real& b) {
	return simd::Max(a, real3::Set(b));
}

real3 Min(const real3& a, const real& b) {
	return simd::Min(a, real3::Set(b));
}
real Max(const real3& a) {
	return simd::Max(a);
}
real Min(const real3& a) {
	return simd::Min(a);
}
real3 Clamp(const real3& a, const real3& clamp_min, const real3& clamp_max) {
	return simd::Max(clamp_min, simd::Min(a, clamp_max));
}
bool operator==(const real3& a, const real3& b) {
	return (a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]);
	// return simd::IsEqual(a, b);
}
real3 Round(const real3& v) {
	return simd::Round(v);
}
bool IsZero(const real3& v) {
	return simd::IsZero(v, C_EPSILON);
}
real3 OrthogonalVector(const real3& v) {
	real3 abs = Abs(v);
	if (abs.x < abs.y) {
		return abs.x < abs.z ? real3(0, v.z, -v.y) : real3(v.y, -v.x, 0);
	} else {
		return abs.y < abs.z ? real3(-v.z, 0, v.x) : real3(v.y, -v.x, 0);
	}
}
real3 UnitOrthogonalVector(const real3& v) {
	return Normalize(OrthogonalVector(v));
}
}
