#include "chrono_parallel/math/sse.h"
#include "chrono_parallel/math/simd.h"
#include "chrono_parallel/math/real4.h"

namespace chrono {

//========================================================
CUDA_HOST_DEVICE real4 operator+(const real4& a, const real4& b) {
	return simd::Add(a, b);
}
CUDA_HOST_DEVICE real4 operator-(const real4& a, const real4& b) {
	return simd::Sub(a, b);
}
CUDA_HOST_DEVICE real4 operator*(const real4& a, const real4& b) {
	return simd::Mul(a, b);
}
CUDA_HOST_DEVICE real4 operator/(const real4& a, const real4& b) {
	return simd::Div(a, b);
}
//========================================================
CUDA_HOST_DEVICE real4 operator+(const real4& a, real b) {
	return simd::Add(a, real4::Set(b));
}
CUDA_HOST_DEVICE real4 operator-(const real4& a, real b) {
	return simd::Sub(a, real4::Set(b));
}
CUDA_HOST_DEVICE real4 operator*(const real4& a, real b) {
	return simd::Mul(a, real4::Set(b));
}
CUDA_HOST_DEVICE real4 operator/(const real4& a, real b) {
	return simd::Div(a, real4::Set(b));
}
CUDA_HOST_DEVICE real4 operator-(const real4& a) {
	return simd::Negate(a);
}
CUDA_HOST_DEVICE real4 Dot4(const real3& v, const real3& v1, const real3& v2, const real3& v3,
		const real3& v4) {
	return simd::Dot4(v, v1, v2, v3, v4);
}
//========================================================
CUDA_HOST_DEVICE quaternion operator+(const quaternion& a, real b) {
	return simd::Add(a, quaternion::Set(b));
}
CUDA_HOST_DEVICE quaternion operator-(const quaternion& a, real b) {
	return simd::Sub(a, quaternion::Set(b));
}
CUDA_HOST_DEVICE quaternion operator*(const quaternion& a, real b) {
	return simd::Mul(a, quaternion::Set(b));
}
CUDA_HOST_DEVICE quaternion operator/(const quaternion& a, real b) {
	return simd::Div(a, quaternion::Set(b));
}
CUDA_HOST_DEVICE quaternion operator-(const quaternion& a) {
	return simd::Negate(a);
}
CUDA_HOST_DEVICE quaternion operator~(const quaternion& a) {
	return simd::change_sign<0, 1, 1, 1>(a);
}
CUDA_HOST_DEVICE quaternion Inv(const quaternion& a) {
	real t1 = Dot(a);
	return (~a) / t1;
}
CUDA_HOST_DEVICE real Dot(const quaternion& v1, const quaternion& v2) {
	return simd::Dot4(v1, v2);
}
CUDA_HOST_DEVICE real Dot(const quaternion& v) {
	return simd::Dot4(v);
}
CUDA_HOST_DEVICE quaternion Mult(const quaternion& a, const quaternion& b) {
	return simd::QuatMult(a, b);
}
CUDA_HOST_DEVICE quaternion Normalize(const quaternion& v) {
	return simd::Normalize(v);
}
}
