/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_SIMD__QUATERNION_H_
#define BT_SIMD__QUATERNION_H_

#include "cbtVector3.h"
#include "cbtQuadWord.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define cbtQuaternionData cbtQuaternionDoubleData
#define cbtQuaternionDataName "cbtQuaternionDoubleData"
#else
#define cbtQuaternionData cbtQuaternionFloatData
#define cbtQuaternionDataName "cbtQuaternionFloatData"
#endif  //BT_USE_DOUBLE_PRECISION

#ifdef BT_USE_SSE

//const __m128 ATTRIBUTE_ALIGNED16(vOnes) = {1.0f, 1.0f, 1.0f, 1.0f};
#define vOnes (_mm_set_ps(1.0f, 1.0f, 1.0f, 1.0f))

#endif

#if defined(BT_USE_SSE)

#define vQInv (_mm_set_ps(+0.0f, -0.0f, -0.0f, -0.0f))
#define vPPPM (_mm_set_ps(-0.0f, +0.0f, +0.0f, +0.0f))

#elif defined(BT_USE_NEON)

const cbtSimdFloat4 ATTRIBUTE_ALIGNED16(vQInv) = {-0.0f, -0.0f, -0.0f, +0.0f};
const cbtSimdFloat4 ATTRIBUTE_ALIGNED16(vPPPM) = {+0.0f, +0.0f, +0.0f, -0.0f};

#endif

/**@brief The cbtQuaternion implements quaternion to perform linear algebra rotations in combination with cbtMatrix3x3, cbtVector3 and cbtTransform. */
class cbtQuaternion : public cbtQuadWord
{
public:
	/**@brief No initialization constructor */
	cbtQuaternion() {}

#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)) || defined(BT_USE_NEON)
	// Set Vector
	SIMD_FORCE_INLINE cbtQuaternion(const cbtSimdFloat4 vec)
	{
		mVec128 = vec;
	}

	// Copy constructor
	SIMD_FORCE_INLINE cbtQuaternion(const cbtQuaternion& rhs)
	{
		mVec128 = rhs.mVec128;
	}

	// Assignment Operator
	SIMD_FORCE_INLINE cbtQuaternion&
	operator=(const cbtQuaternion& v)
	{
		mVec128 = v.mVec128;

		return *this;
	}

#endif

	//		template <typename cbtScalar>
	//		explicit Quaternion(const cbtScalar *v) : Tuple4<cbtScalar>(v) {}
	/**@brief Constructor from scalars */
	cbtQuaternion(const cbtScalar& _x, const cbtScalar& _y, const cbtScalar& _z, const cbtScalar& _w)
		: cbtQuadWord(_x, _y, _z, _w)
	{
	}
	/**@brief Axis angle Constructor
   * @param axis The axis which the rotation is around
   * @param angle The magnitude of the rotation around the angle (Radians) */
	cbtQuaternion(const cbtVector3& _axis, const cbtScalar& _angle)
	{
		setRotation(_axis, _angle);
	}
	/**@brief Constructor from Euler angles
   * @param yaw Angle around Y unless BT_EULER_DEFAULT_ZYX defined then Z
   * @param pitch Angle around X unless BT_EULER_DEFAULT_ZYX defined then Y
   * @param roll Angle around Z unless BT_EULER_DEFAULT_ZYX defined then X */
	cbtQuaternion(const cbtScalar& yaw, const cbtScalar& pitch, const cbtScalar& roll)
	{
#ifndef BT_EULER_DEFAULT_ZYX
		setEuler(yaw, pitch, roll);
#else
		setEulerZYX(yaw, pitch, roll);
#endif
	}
	/**@brief Set the rotation using axis angle notation 
   * @param axis The axis around which to rotate
   * @param angle The magnitude of the rotation in Radians */
	void setRotation(const cbtVector3& axis, const cbtScalar& _angle)
	{
		cbtScalar d = axis.length();
		cbtAssert(d != cbtScalar(0.0));
		cbtScalar s = cbtSin(_angle * cbtScalar(0.5)) / d;
		setValue(axis.x() * s, axis.y() * s, axis.z() * s,
				 cbtCos(_angle * cbtScalar(0.5)));
	}
	/**@brief Set the quaternion using Euler angles
   * @param yaw Angle around Y
   * @param pitch Angle around X
   * @param roll Angle around Z */
	void setEuler(const cbtScalar& yaw, const cbtScalar& pitch, const cbtScalar& roll)
	{
		cbtScalar halfYaw = cbtScalar(yaw) * cbtScalar(0.5);
		cbtScalar halfPitch = cbtScalar(pitch) * cbtScalar(0.5);
		cbtScalar halfRoll = cbtScalar(roll) * cbtScalar(0.5);
		cbtScalar cosYaw = cbtCos(halfYaw);
		cbtScalar sinYaw = cbtSin(halfYaw);
		cbtScalar cosPitch = cbtCos(halfPitch);
		cbtScalar sinPitch = cbtSin(halfPitch);
		cbtScalar cosRoll = cbtCos(halfRoll);
		cbtScalar sinRoll = cbtSin(halfRoll);
		setValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
				 cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
				 sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
				 cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
	}
	/**@brief Set the quaternion using euler angles 
   * @param yaw Angle around Z
   * @param pitch Angle around Y
   * @param roll Angle around X */
	void setEulerZYX(const cbtScalar& yawZ, const cbtScalar& pitchY, const cbtScalar& rollX)
	{
		cbtScalar halfYaw = cbtScalar(yawZ) * cbtScalar(0.5);
		cbtScalar halfPitch = cbtScalar(pitchY) * cbtScalar(0.5);
		cbtScalar halfRoll = cbtScalar(rollX) * cbtScalar(0.5);
		cbtScalar cosYaw = cbtCos(halfYaw);
		cbtScalar sinYaw = cbtSin(halfYaw);
		cbtScalar cosPitch = cbtCos(halfPitch);
		cbtScalar sinPitch = cbtSin(halfPitch);
		cbtScalar cosRoll = cbtCos(halfRoll);
		cbtScalar sinRoll = cbtSin(halfRoll);
		setValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,   //x
				 cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,   //y
				 cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,   //z
				 cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);  //formerly yzx
	}

	/**@brief Get the euler angles from this quaternion
	   * @param yaw Angle around Z
	   * @param pitch Angle around Y
	   * @param roll Angle around X */
	void getEulerZYX(cbtScalar& yawZ, cbtScalar& pitchY, cbtScalar& rollX) const
	{
		cbtScalar squ;
		cbtScalar sqx;
		cbtScalar sqy;
		cbtScalar sqz;
		cbtScalar sarg;
		sqx = m_floats[0] * m_floats[0];
		sqy = m_floats[1] * m_floats[1];
		sqz = m_floats[2] * m_floats[2];
		squ = m_floats[3] * m_floats[3];
		sarg = cbtScalar(-2.) * (m_floats[0] * m_floats[2] - m_floats[3] * m_floats[1]);

		// If the pitch angle is PI/2 or -PI/2, we can only compute
		// the sum roll + yaw.  However, any combination that gives
		// the right sum will produce the correct orientation, so we
		// set rollX = 0 and compute yawZ.
		if (sarg <= -cbtScalar(0.99999))
		{
			pitchY = cbtScalar(-0.5) * SIMD_PI;
			rollX = 0;
			yawZ = cbtScalar(2) * cbtAtan2(m_floats[0], -m_floats[1]);
		}
		else if (sarg >= cbtScalar(0.99999))
		{
			pitchY = cbtScalar(0.5) * SIMD_PI;
			rollX = 0;
			yawZ = cbtScalar(2) * cbtAtan2(-m_floats[0], m_floats[1]);
		}
		else
		{
			pitchY = cbtAsin(sarg);
			rollX = cbtAtan2(2 * (m_floats[1] * m_floats[2] + m_floats[3] * m_floats[0]), squ - sqx - sqy + sqz);
			yawZ = cbtAtan2(2 * (m_floats[0] * m_floats[1] + m_floats[3] * m_floats[2]), squ + sqx - sqy - sqz);
		}
	}

	/**@brief Add two quaternions
   * @param q The quaternion to add to this one */
	SIMD_FORCE_INLINE cbtQuaternion& operator+=(const cbtQuaternion& q)
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		mVec128 = _mm_add_ps(mVec128, q.mVec128);
#elif defined(BT_USE_NEON)
		mVec128 = vaddq_f32(mVec128, q.mVec128);
#else
		m_floats[0] += q.x();
		m_floats[1] += q.y();
		m_floats[2] += q.z();
		m_floats[3] += q.m_floats[3];
#endif
		return *this;
	}

	/**@brief Subtract out a quaternion
   * @param q The quaternion to subtract from this one */
	cbtQuaternion& operator-=(const cbtQuaternion& q)
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		mVec128 = _mm_sub_ps(mVec128, q.mVec128);
#elif defined(BT_USE_NEON)
		mVec128 = vsubq_f32(mVec128, q.mVec128);
#else
		m_floats[0] -= q.x();
		m_floats[1] -= q.y();
		m_floats[2] -= q.z();
		m_floats[3] -= q.m_floats[3];
#endif
		return *this;
	}

	/**@brief Scale this quaternion
   * @param s The scalar to scale by */
	cbtQuaternion& operator*=(const cbtScalar& s)
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		__m128 vs = _mm_load_ss(&s);  //	(S 0 0 0)
		vs = bt_pshufd_ps(vs, 0);     //	(S S S S)
		mVec128 = _mm_mul_ps(mVec128, vs);
#elif defined(BT_USE_NEON)
		mVec128 = vmulq_n_f32(mVec128, s);
#else
		m_floats[0] *= s;
		m_floats[1] *= s;
		m_floats[2] *= s;
		m_floats[3] *= s;
#endif
		return *this;
	}

	/**@brief Multiply this quaternion by q on the right
   * @param q The other quaternion 
   * Equivilant to this = this * q */
	cbtQuaternion& operator*=(const cbtQuaternion& q)
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		__m128 vQ2 = q.get128();

		__m128 A1 = bt_pshufd_ps(mVec128, BT_SHUFFLE(0, 1, 2, 0));
		__m128 B1 = bt_pshufd_ps(vQ2, BT_SHUFFLE(3, 3, 3, 0));

		A1 = A1 * B1;

		__m128 A2 = bt_pshufd_ps(mVec128, BT_SHUFFLE(1, 2, 0, 1));
		__m128 B2 = bt_pshufd_ps(vQ2, BT_SHUFFLE(2, 0, 1, 1));

		A2 = A2 * B2;

		B1 = bt_pshufd_ps(mVec128, BT_SHUFFLE(2, 0, 1, 2));
		B2 = bt_pshufd_ps(vQ2, BT_SHUFFLE(1, 2, 0, 2));

		B1 = B1 * B2;  //	A3 *= B3

		mVec128 = bt_splat_ps(mVec128, 3);  //	A0
		mVec128 = mVec128 * vQ2;            //	A0 * B0

		A1 = A1 + A2;                //	AB12
		mVec128 = mVec128 - B1;      //	AB03 = AB0 - AB3
		A1 = _mm_xor_ps(A1, vPPPM);  //	change sign of the last element
		mVec128 = mVec128 + A1;      //	AB03 + AB12

#elif defined(BT_USE_NEON)

		float32x4_t vQ1 = mVec128;
		float32x4_t vQ2 = q.get128();
		float32x4_t A0, A1, B1, A2, B2, A3, B3;
		float32x2_t vQ1zx, vQ2wx, vQ1yz, vQ2zx, vQ2yz, vQ2xz;

		{
			float32x2x2_t tmp;
			tmp = vtrn_f32(vget_high_f32(vQ1), vget_low_f32(vQ1));  // {z x}, {w y}
			vQ1zx = tmp.val[0];

			tmp = vtrn_f32(vget_high_f32(vQ2), vget_low_f32(vQ2));  // {z x}, {w y}
			vQ2zx = tmp.val[0];
		}
		vQ2wx = vext_f32(vget_high_f32(vQ2), vget_low_f32(vQ2), 1);

		vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

		vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
		vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

		A1 = vcombine_f32(vget_low_f32(vQ1), vQ1zx);                     // X Y  z x
		B1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ2), 1), vQ2wx);  // W W  W X

		A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
		B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

		A3 = vcombine_f32(vQ1zx, vQ1yz);  // Z X Y Z
		B3 = vcombine_f32(vQ2yz, vQ2xz);  // Y Z x z

		A1 = vmulq_f32(A1, B1);
		A2 = vmulq_f32(A2, B2);
		A3 = vmulq_f32(A3, B3);                           //	A3 *= B3
		A0 = vmulq_lane_f32(vQ2, vget_high_f32(vQ1), 1);  //	A0 * B0

		A1 = vaddq_f32(A1, A2);  //	AB12 = AB1 + AB2
		A0 = vsubq_f32(A0, A3);  //	AB03 = AB0 - AB3

		//	change the sign of the last element
		A1 = (cbtSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);
		A0 = vaddq_f32(A0, A1);  //	AB03 + AB12

		mVec128 = A0;
#else
		setValue(
			m_floats[3] * q.x() + m_floats[0] * q.m_floats[3] + m_floats[1] * q.z() - m_floats[2] * q.y(),
			m_floats[3] * q.y() + m_floats[1] * q.m_floats[3] + m_floats[2] * q.x() - m_floats[0] * q.z(),
			m_floats[3] * q.z() + m_floats[2] * q.m_floats[3] + m_floats[0] * q.y() - m_floats[1] * q.x(),
			m_floats[3] * q.m_floats[3] - m_floats[0] * q.x() - m_floats[1] * q.y() - m_floats[2] * q.z());
#endif
		return *this;
	}
	/**@brief Return the dot product between this quaternion and another
   * @param q The other quaternion */
	cbtScalar dot(const cbtQuaternion& q) const
	{
#if defined BT_USE_SIMD_VECTOR3 && defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		__m128 vd;

		vd = _mm_mul_ps(mVec128, q.mVec128);

		__m128 t = _mm_movehl_ps(vd, vd);
		vd = _mm_add_ps(vd, t);
		t = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, t);

		return _mm_cvtss_f32(vd);
#elif defined(BT_USE_NEON)
		float32x4_t vd = vmulq_f32(mVec128, q.mVec128);
		float32x2_t x = vpadd_f32(vget_low_f32(vd), vget_high_f32(vd));
		x = vpadd_f32(x, x);
		return vget_lane_f32(x, 0);
#else
		return m_floats[0] * q.x() +
			   m_floats[1] * q.y() +
			   m_floats[2] * q.z() +
			   m_floats[3] * q.m_floats[3];
#endif
	}

	/**@brief Return the length squared of the quaternion */
	cbtScalar length2() const
	{
		return dot(*this);
	}

	/**@brief Return the length of the quaternion */
	cbtScalar length() const
	{
		return cbtSqrt(length2());
	}
	cbtQuaternion& safeNormalize()
	{
		cbtScalar l2 = length2();
		if (l2 > SIMD_EPSILON)
		{
			normalize();
		}
		return *this;
	}
	/**@brief Normalize the quaternion 
   * Such that x^2 + y^2 + z^2 +w^2 = 1 */
	cbtQuaternion& normalize()
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		__m128 vd;

		vd = _mm_mul_ps(mVec128, mVec128);

		__m128 t = _mm_movehl_ps(vd, vd);
		vd = _mm_add_ps(vd, t);
		t = _mm_shuffle_ps(vd, vd, 0x55);
		vd = _mm_add_ss(vd, t);

		vd = _mm_sqrt_ss(vd);
		vd = _mm_div_ss(vOnes, vd);
		vd = bt_pshufd_ps(vd, 0);  // splat
		mVec128 = _mm_mul_ps(mVec128, vd);

		return *this;
#else
		return *this /= length();
#endif
	}

	/**@brief Return a scaled version of this quaternion
   * @param s The scale factor */
	SIMD_FORCE_INLINE cbtQuaternion
	operator*(const cbtScalar& s) const
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		__m128 vs = _mm_load_ss(&s);  //	(S 0 0 0)
		vs = bt_pshufd_ps(vs, 0x00);  //	(S S S S)

		return cbtQuaternion(_mm_mul_ps(mVec128, vs));
#elif defined(BT_USE_NEON)
		return cbtQuaternion(vmulq_n_f32(mVec128, s));
#else
		return cbtQuaternion(x() * s, y() * s, z() * s, m_floats[3] * s);
#endif
	}

	/**@brief Return an inversely scaled versionof this quaternion
   * @param s The inverse scale factor */
	cbtQuaternion operator/(const cbtScalar& s) const
	{
		cbtAssert(s != cbtScalar(0.0));
		return *this * (cbtScalar(1.0) / s);
	}

	/**@brief Inversely scale this quaternion
   * @param s The scale factor */
	cbtQuaternion& operator/=(const cbtScalar& s)
	{
		cbtAssert(s != cbtScalar(0.0));
		return *this *= cbtScalar(1.0) / s;
	}

	/**@brief Return a normalized version of this quaternion */
	cbtQuaternion normalized() const
	{
		return *this / length();
	}
	/**@brief Return the ***half*** angle between this quaternion and the other
   * @param q The other quaternion */
	cbtScalar angle(const cbtQuaternion& q) const
	{
		cbtScalar s = cbtSqrt(length2() * q.length2());
		cbtAssert(s != cbtScalar(0.0));
		return cbtAcos(dot(q) / s);
	}

	/**@brief Return the angle between this quaternion and the other along the shortest path
	* @param q The other quaternion */
	cbtScalar angleShortestPath(const cbtQuaternion& q) const
	{
		cbtScalar s = cbtSqrt(length2() * q.length2());
		cbtAssert(s != cbtScalar(0.0));
		if (dot(q) < 0)  // Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
			return cbtAcos(dot(-q) / s) * cbtScalar(2.0);
		else
			return cbtAcos(dot(q) / s) * cbtScalar(2.0);
	}

	/**@brief Return the angle [0, 2Pi] of rotation represented by this quaternion */
	cbtScalar getAngle() const
	{
		cbtScalar s = cbtScalar(2.) * cbtAcos(m_floats[3]);
		return s;
	}

	/**@brief Return the angle [0, Pi] of rotation represented by this quaternion along the shortest path */
	cbtScalar getAngleShortestPath() const
	{
		cbtScalar s;
		if (m_floats[3] >= 0)
			s = cbtScalar(2.) * cbtAcos(m_floats[3]);
		else
			s = cbtScalar(2.) * cbtAcos(-m_floats[3]);
		return s;
	}

	/**@brief Return the axis of the rotation represented by this quaternion */
	cbtVector3 getAxis() const
	{
		cbtScalar s_squared = 1.f - m_floats[3] * m_floats[3];

		if (s_squared < cbtScalar(10.) * SIMD_EPSILON)  //Check for divide by zero
			return cbtVector3(1.0, 0.0, 0.0);           // Arbitrary
		cbtScalar s = 1.f / cbtSqrt(s_squared);
		return cbtVector3(m_floats[0] * s, m_floats[1] * s, m_floats[2] * s);
	}

	/**@brief Return the inverse of this quaternion */
	cbtQuaternion inverse() const
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		return cbtQuaternion(_mm_xor_ps(mVec128, vQInv));
#elif defined(BT_USE_NEON)
		return cbtQuaternion((cbtSimdFloat4)veorq_s32((int32x4_t)mVec128, (int32x4_t)vQInv));
#else
		return cbtQuaternion(-m_floats[0], -m_floats[1], -m_floats[2], m_floats[3]);
#endif
	}

	/**@brief Return the sum of this quaternion and the other 
   * @param q2 The other quaternion */
	SIMD_FORCE_INLINE cbtQuaternion
	operator+(const cbtQuaternion& q2) const
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		return cbtQuaternion(_mm_add_ps(mVec128, q2.mVec128));
#elif defined(BT_USE_NEON)
		return cbtQuaternion(vaddq_f32(mVec128, q2.mVec128));
#else
		const cbtQuaternion& q1 = *this;
		return cbtQuaternion(q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z(), q1.m_floats[3] + q2.m_floats[3]);
#endif
	}

	/**@brief Return the difference between this quaternion and the other 
   * @param q2 The other quaternion */
	SIMD_FORCE_INLINE cbtQuaternion
	operator-(const cbtQuaternion& q2) const
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		return cbtQuaternion(_mm_sub_ps(mVec128, q2.mVec128));
#elif defined(BT_USE_NEON)
		return cbtQuaternion(vsubq_f32(mVec128, q2.mVec128));
#else
		const cbtQuaternion& q1 = *this;
		return cbtQuaternion(q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z(), q1.m_floats[3] - q2.m_floats[3]);
#endif
	}

	/**@brief Return the negative of this quaternion 
   * This simply negates each element */
	SIMD_FORCE_INLINE cbtQuaternion operator-() const
	{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
		return cbtQuaternion(_mm_xor_ps(mVec128, btvMzeroMask));
#elif defined(BT_USE_NEON)
		return cbtQuaternion((cbtSimdFloat4)veorq_s32((int32x4_t)mVec128, (int32x4_t)btvMzeroMask));
#else
		const cbtQuaternion& q2 = *this;
		return cbtQuaternion(-q2.x(), -q2.y(), -q2.z(), -q2.m_floats[3]);
#endif
	}
	/**@todo document this and it's use */
	SIMD_FORCE_INLINE cbtQuaternion farthest(const cbtQuaternion& qd) const
	{
		cbtQuaternion diff, sum;
		diff = *this - qd;
		sum = *this + qd;
		if (diff.dot(diff) > sum.dot(sum))
			return qd;
		return (-qd);
	}

	/**@todo document this and it's use */
	SIMD_FORCE_INLINE cbtQuaternion nearest(const cbtQuaternion& qd) const
	{
		cbtQuaternion diff, sum;
		diff = *this - qd;
		sum = *this + qd;
		if (diff.dot(diff) < sum.dot(sum))
			return qd;
		return (-qd);
	}

	/**@brief Return the quaternion which is the result of Spherical Linear Interpolation between this and the other quaternion
   * @param q The other quaternion to interpolate with 
   * @param t The ratio between this and q to interpolate.  If t = 0 the result is this, if t=1 the result is q.
   * Slerp interpolates assuming constant velocity.  */
	cbtQuaternion slerp(const cbtQuaternion& q, const cbtScalar& t) const
	{
		const cbtScalar magnitude = cbtSqrt(length2() * q.length2());
		cbtAssert(magnitude > cbtScalar(0));

		const cbtScalar product = dot(q) / magnitude;
		const cbtScalar absproduct = cbtFabs(product);

		if (absproduct < cbtScalar(1.0 - SIMD_EPSILON))
		{
			// Take care of long angle case see http://en.wikipedia.org/wiki/Slerp
			const cbtScalar theta = cbtAcos(absproduct);
			const cbtScalar d = cbtSin(theta);
			cbtAssert(d > cbtScalar(0));

			const cbtScalar sign = (product < 0) ? cbtScalar(-1) : cbtScalar(1);
			const cbtScalar s0 = cbtSin((cbtScalar(1.0) - t) * theta) / d;
			const cbtScalar s1 = cbtSin(sign * t * theta) / d;

			return cbtQuaternion(
				(m_floats[0] * s0 + q.x() * s1),
				(m_floats[1] * s0 + q.y() * s1),
				(m_floats[2] * s0 + q.z() * s1),
				(m_floats[3] * s0 + q.w() * s1));
		}
		else
		{
			return *this;
		}
	}

	static const cbtQuaternion& getIdentity()
	{
		static const cbtQuaternion identityQuat(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.), cbtScalar(1.));
		return identityQuat;
	}

	SIMD_FORCE_INLINE const cbtScalar& getW() const { return m_floats[3]; }

	SIMD_FORCE_INLINE void serialize(struct cbtQuaternionData& dataOut) const;

	SIMD_FORCE_INLINE void deSerialize(const struct cbtQuaternionFloatData& dataIn);

	SIMD_FORCE_INLINE void deSerialize(const struct cbtQuaternionDoubleData& dataIn);

	SIMD_FORCE_INLINE void serializeFloat(struct cbtQuaternionFloatData& dataOut) const;

	SIMD_FORCE_INLINE void deSerializeFloat(const struct cbtQuaternionFloatData& dataIn);

	SIMD_FORCE_INLINE void serializeDouble(struct cbtQuaternionDoubleData& dataOut) const;

	SIMD_FORCE_INLINE void deSerializeDouble(const struct cbtQuaternionDoubleData& dataIn);
};

/**@brief Return the product of two quaternions */
SIMD_FORCE_INLINE cbtQuaternion
operator*(const cbtQuaternion& q1, const cbtQuaternion& q2)
{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
	__m128 vQ1 = q1.get128();
	__m128 vQ2 = q2.get128();
	__m128 A0, A1, B1, A2, B2;

	A1 = bt_pshufd_ps(vQ1, BT_SHUFFLE(0, 1, 2, 0));  // X Y  z x     //      vtrn
	B1 = bt_pshufd_ps(vQ2, BT_SHUFFLE(3, 3, 3, 0));  // W W  W X     // vdup vext

	A1 = A1 * B1;

	A2 = bt_pshufd_ps(vQ1, BT_SHUFFLE(1, 2, 0, 1));  // Y Z  X Y     // vext
	B2 = bt_pshufd_ps(vQ2, BT_SHUFFLE(2, 0, 1, 1));  // z x  Y Y     // vtrn vdup

	A2 = A2 * B2;

	B1 = bt_pshufd_ps(vQ1, BT_SHUFFLE(2, 0, 1, 2));  // z x Y Z      // vtrn vext
	B2 = bt_pshufd_ps(vQ2, BT_SHUFFLE(1, 2, 0, 2));  // Y Z x z      // vext vtrn

	B1 = B1 * B2;  //	A3 *= B3

	A0 = bt_splat_ps(vQ1, 3);  //	A0
	A0 = A0 * vQ2;             //	A0 * B0

	A1 = A1 + A2;  //	AB12
	A0 = A0 - B1;  //	AB03 = AB0 - AB3

	A1 = _mm_xor_ps(A1, vPPPM);  //	change sign of the last element
	A0 = A0 + A1;                //	AB03 + AB12

	return cbtQuaternion(A0);

#elif defined(BT_USE_NEON)

	float32x4_t vQ1 = q1.get128();
	float32x4_t vQ2 = q2.get128();
	float32x4_t A0, A1, B1, A2, B2, A3, B3;
	float32x2_t vQ1zx, vQ2wx, vQ1yz, vQ2zx, vQ2yz, vQ2xz;

	{
		float32x2x2_t tmp;
		tmp = vtrn_f32(vget_high_f32(vQ1), vget_low_f32(vQ1));  // {z x}, {w y}
		vQ1zx = tmp.val[0];

		tmp = vtrn_f32(vget_high_f32(vQ2), vget_low_f32(vQ2));  // {z x}, {w y}
		vQ2zx = tmp.val[0];
	}
	vQ2wx = vext_f32(vget_high_f32(vQ2), vget_low_f32(vQ2), 1);

	vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

	vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
	vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

	A1 = vcombine_f32(vget_low_f32(vQ1), vQ1zx);                     // X Y  z x
	B1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ2), 1), vQ2wx);  // W W  W X

	A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
	B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

	A3 = vcombine_f32(vQ1zx, vQ1yz);  // Z X Y Z
	B3 = vcombine_f32(vQ2yz, vQ2xz);  // Y Z x z

	A1 = vmulq_f32(A1, B1);
	A2 = vmulq_f32(A2, B2);
	A3 = vmulq_f32(A3, B3);                           //	A3 *= B3
	A0 = vmulq_lane_f32(vQ2, vget_high_f32(vQ1), 1);  //	A0 * B0

	A1 = vaddq_f32(A1, A2);  //	AB12 = AB1 + AB2
	A0 = vsubq_f32(A0, A3);  //	AB03 = AB0 - AB3

	//	change the sign of the last element
	A1 = (cbtSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);
	A0 = vaddq_f32(A0, A1);  //	AB03 + AB12

	return cbtQuaternion(A0);

#else
	return cbtQuaternion(
		q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
		q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
		q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
		q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z());
#endif
}

SIMD_FORCE_INLINE cbtQuaternion
operator*(const cbtQuaternion& q, const cbtVector3& w)
{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
	__m128 vQ1 = q.get128();
	__m128 vQ2 = w.get128();
	__m128 A1, B1, A2, B2, A3, B3;

	A1 = bt_pshufd_ps(vQ1, BT_SHUFFLE(3, 3, 3, 0));
	B1 = bt_pshufd_ps(vQ2, BT_SHUFFLE(0, 1, 2, 0));

	A1 = A1 * B1;

	A2 = bt_pshufd_ps(vQ1, BT_SHUFFLE(1, 2, 0, 1));
	B2 = bt_pshufd_ps(vQ2, BT_SHUFFLE(2, 0, 1, 1));

	A2 = A2 * B2;

	A3 = bt_pshufd_ps(vQ1, BT_SHUFFLE(2, 0, 1, 2));
	B3 = bt_pshufd_ps(vQ2, BT_SHUFFLE(1, 2, 0, 2));

	A3 = A3 * B3;  //	A3 *= B3

	A1 = A1 + A2;                //	AB12
	A1 = _mm_xor_ps(A1, vPPPM);  //	change sign of the last element
	A1 = A1 - A3;                //	AB123 = AB12 - AB3

	return cbtQuaternion(A1);

#elif defined(BT_USE_NEON)

	float32x4_t vQ1 = q.get128();
	float32x4_t vQ2 = w.get128();
	float32x4_t A1, B1, A2, B2, A3, B3;
	float32x2_t vQ1wx, vQ2zx, vQ1yz, vQ2yz, vQ1zx, vQ2xz;

	vQ1wx = vext_f32(vget_high_f32(vQ1), vget_low_f32(vQ1), 1);
	{
		float32x2x2_t tmp;

		tmp = vtrn_f32(vget_high_f32(vQ2), vget_low_f32(vQ2));  // {z x}, {w y}
		vQ2zx = tmp.val[0];

		tmp = vtrn_f32(vget_high_f32(vQ1), vget_low_f32(vQ1));  // {z x}, {w y}
		vQ1zx = tmp.val[0];
	}

	vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

	vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
	vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

	A1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ1), 1), vQ1wx);  // W W  W X
	B1 = vcombine_f32(vget_low_f32(vQ2), vQ2zx);                     // X Y  z x

	A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
	B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

	A3 = vcombine_f32(vQ1zx, vQ1yz);  // Z X Y Z
	B3 = vcombine_f32(vQ2yz, vQ2xz);  // Y Z x z

	A1 = vmulq_f32(A1, B1);
	A2 = vmulq_f32(A2, B2);
	A3 = vmulq_f32(A3, B3);  //	A3 *= B3

	A1 = vaddq_f32(A1, A2);  //	AB12 = AB1 + AB2

	//	change the sign of the last element
	A1 = (cbtSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);

	A1 = vsubq_f32(A1, A3);  //	AB123 = AB12 - AB3

	return cbtQuaternion(A1);

#else
	return cbtQuaternion(
		q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
		q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
		q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z());
#endif
}

SIMD_FORCE_INLINE cbtQuaternion
operator*(const cbtVector3& w, const cbtQuaternion& q)
{
#if defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
	__m128 vQ1 = w.get128();
	__m128 vQ2 = q.get128();
	__m128 A1, B1, A2, B2, A3, B3;

	A1 = bt_pshufd_ps(vQ1, BT_SHUFFLE(0, 1, 2, 0));  // X Y  z x
	B1 = bt_pshufd_ps(vQ2, BT_SHUFFLE(3, 3, 3, 0));  // W W  W X

	A1 = A1 * B1;

	A2 = bt_pshufd_ps(vQ1, BT_SHUFFLE(1, 2, 0, 1));
	B2 = bt_pshufd_ps(vQ2, BT_SHUFFLE(2, 0, 1, 1));

	A2 = A2 * B2;

	A3 = bt_pshufd_ps(vQ1, BT_SHUFFLE(2, 0, 1, 2));
	B3 = bt_pshufd_ps(vQ2, BT_SHUFFLE(1, 2, 0, 2));

	A3 = A3 * B3;  //	A3 *= B3

	A1 = A1 + A2;                //	AB12
	A1 = _mm_xor_ps(A1, vPPPM);  //	change sign of the last element
	A1 = A1 - A3;                //	AB123 = AB12 - AB3

	return cbtQuaternion(A1);

#elif defined(BT_USE_NEON)

	float32x4_t vQ1 = w.get128();
	float32x4_t vQ2 = q.get128();
	float32x4_t A1, B1, A2, B2, A3, B3;
	float32x2_t vQ1zx, vQ2wx, vQ1yz, vQ2zx, vQ2yz, vQ2xz;

	{
		float32x2x2_t tmp;

		tmp = vtrn_f32(vget_high_f32(vQ1), vget_low_f32(vQ1));  // {z x}, {w y}
		vQ1zx = tmp.val[0];

		tmp = vtrn_f32(vget_high_f32(vQ2), vget_low_f32(vQ2));  // {z x}, {w y}
		vQ2zx = tmp.val[0];
	}
	vQ2wx = vext_f32(vget_high_f32(vQ2), vget_low_f32(vQ2), 1);

	vQ1yz = vext_f32(vget_low_f32(vQ1), vget_high_f32(vQ1), 1);

	vQ2yz = vext_f32(vget_low_f32(vQ2), vget_high_f32(vQ2), 1);
	vQ2xz = vext_f32(vQ2zx, vQ2zx, 1);

	A1 = vcombine_f32(vget_low_f32(vQ1), vQ1zx);                     // X Y  z x
	B1 = vcombine_f32(vdup_lane_f32(vget_high_f32(vQ2), 1), vQ2wx);  // W W  W X

	A2 = vcombine_f32(vQ1yz, vget_low_f32(vQ1));
	B2 = vcombine_f32(vQ2zx, vdup_lane_f32(vget_low_f32(vQ2), 1));

	A3 = vcombine_f32(vQ1zx, vQ1yz);  // Z X Y Z
	B3 = vcombine_f32(vQ2yz, vQ2xz);  // Y Z x z

	A1 = vmulq_f32(A1, B1);
	A2 = vmulq_f32(A2, B2);
	A3 = vmulq_f32(A3, B3);  //	A3 *= B3

	A1 = vaddq_f32(A1, A2);  //	AB12 = AB1 + AB2

	//	change the sign of the last element
	A1 = (cbtSimdFloat4)veorq_s32((int32x4_t)A1, (int32x4_t)vPPPM);

	A1 = vsubq_f32(A1, A3);  //	AB123 = AB12 - AB3

	return cbtQuaternion(A1);

#else
	return cbtQuaternion(
		+w.x() * q.w() + w.y() * q.z() - w.z() * q.y(),
		+w.y() * q.w() + w.z() * q.x() - w.x() * q.z(),
		+w.z() * q.w() + w.x() * q.y() - w.y() * q.x(),
		-w.x() * q.x() - w.y() * q.y() - w.z() * q.z());
#endif
}

/**@brief Calculate the dot product between two quaternions */
SIMD_FORCE_INLINE cbtScalar
dot(const cbtQuaternion& q1, const cbtQuaternion& q2)
{
	return q1.dot(q2);
}

/**@brief Return the length of a quaternion */
SIMD_FORCE_INLINE cbtScalar
length(const cbtQuaternion& q)
{
	return q.length();
}

/**@brief Return the angle between two quaternions*/
SIMD_FORCE_INLINE cbtScalar
cbtAngle(const cbtQuaternion& q1, const cbtQuaternion& q2)
{
	return q1.angle(q2);
}

/**@brief Return the inverse of a quaternion*/
SIMD_FORCE_INLINE cbtQuaternion
inverse(const cbtQuaternion& q)
{
	return q.inverse();
}

/**@brief Return the result of spherical linear interpolation betwen two quaternions 
 * @param q1 The first quaternion
 * @param q2 The second quaternion 
 * @param t The ration between q1 and q2.  t = 0 return q1, t=1 returns q2 
 * Slerp assumes constant velocity between positions. */
SIMD_FORCE_INLINE cbtQuaternion
slerp(const cbtQuaternion& q1, const cbtQuaternion& q2, const cbtScalar& t)
{
	return q1.slerp(q2, t);
}

SIMD_FORCE_INLINE cbtVector3
quatRotate(const cbtQuaternion& rotation, const cbtVector3& v)
{
	cbtQuaternion q = rotation * v;
	q *= rotation.inverse();
#if defined BT_USE_SIMD_VECTOR3 && defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)
	return cbtVector3(_mm_and_ps(q.get128(), btvFFF0fMask));
#elif defined(BT_USE_NEON)
	return cbtVector3((float32x4_t)vandq_s32((int32x4_t)q.get128(), btvFFF0Mask));
#else
	return cbtVector3(q.getX(), q.getY(), q.getZ());
#endif
}

SIMD_FORCE_INLINE cbtQuaternion
shortestArcQuat(const cbtVector3& v0, const cbtVector3& v1)  // Game Programming Gems 2.10. make sure v0,v1 are normalized
{
	cbtVector3 c = v0.cross(v1);
	cbtScalar d = v0.dot(v1);

	if (d < -1.0 + SIMD_EPSILON)
	{
		cbtVector3 n, unused;
		cbtPlaneSpace1(v0, n, unused);
		return cbtQuaternion(n.x(), n.y(), n.z(), 0.0f);  // just pick any vector that is orthogonal to v0
	}

	cbtScalar s = cbtSqrt((1.0f + d) * 2.0f);
	cbtScalar rs = 1.0f / s;

	return cbtQuaternion(c.getX() * rs, c.getY() * rs, c.getZ() * rs, s * 0.5f);
}

SIMD_FORCE_INLINE cbtQuaternion
shortestArcQuatNormalize2(cbtVector3& v0, cbtVector3& v1)
{
	v0.normalize();
	v1.normalize();
	return shortestArcQuat(v0, v1);
}

struct cbtQuaternionFloatData
{
	float m_floats[4];
};

struct cbtQuaternionDoubleData
{
	double m_floats[4];
};

SIMD_FORCE_INLINE void cbtQuaternion::serializeFloat(struct cbtQuaternionFloatData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i < 4; i++)
		dataOut.m_floats[i] = float(m_floats[i]);
}

SIMD_FORCE_INLINE void cbtQuaternion::deSerializeFloat(const struct cbtQuaternionFloatData& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = cbtScalar(dataIn.m_floats[i]);
}

SIMD_FORCE_INLINE void cbtQuaternion::serializeDouble(struct cbtQuaternionDoubleData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i < 4; i++)
		dataOut.m_floats[i] = double(m_floats[i]);
}

SIMD_FORCE_INLINE void cbtQuaternion::deSerializeDouble(const struct cbtQuaternionDoubleData& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = cbtScalar(dataIn.m_floats[i]);
}

SIMD_FORCE_INLINE void cbtQuaternion::serialize(struct cbtQuaternionData& dataOut) const
{
	///could also do a memcpy, check if it is worth it
	for (int i = 0; i < 4; i++)
		dataOut.m_floats[i] = m_floats[i];
}

SIMD_FORCE_INLINE void cbtQuaternion::deSerialize(const struct cbtQuaternionFloatData& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = (cbtScalar)dataIn.m_floats[i];
}

SIMD_FORCE_INLINE void cbtQuaternion::deSerialize(const struct cbtQuaternionDoubleData& dataIn)
{
	for (int i = 0; i < 4; i++)
		m_floats[i] = (cbtScalar)dataIn.m_floats[i];
}

#endif  //BT_SIMD__QUATERNION_H_
