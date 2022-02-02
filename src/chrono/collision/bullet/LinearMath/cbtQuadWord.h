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

#ifndef BT_SIMD_QUADWORD_H
#define BT_SIMD_QUADWORD_H

#include "cbtScalar.h"
#include "cbtMinMax.h"

#if defined(__CELLOS_LV2) && defined(__SPU__)
#include <altivec.h>
#endif

/**@brief The cbtQuadWord class is base class for cbtVector3 and cbtQuaternion. 
 * Some issues under PS3 Linux with IBM 2.1 SDK, gcc compiler prevent from using aligned quadword.
 */
#ifndef USE_LIBSPE2
ATTRIBUTE_ALIGNED16(class)
cbtQuadWord
#else
class cbtQuadWord
#endif
{
protected:
#if defined(__SPU__) && defined(__CELLOS_LV2__)
	union {
		vec_float4 mVec128;
		cbtScalar m_floats[4];
	};

public:
	vec_float4 get128() const
	{
		return mVec128;
	}

protected:
#else  //__CELLOS_LV2__ __SPU__

#if defined(BT_USE_SSE) || defined(BT_USE_NEON)
	union {
		cbtSimdFloat4 mVec128;
		cbtScalar m_floats[4];
	};

public:
	SIMD_FORCE_INLINE cbtSimdFloat4 get128() const
	{
		return mVec128;
	}
	SIMD_FORCE_INLINE void set128(cbtSimdFloat4 v128)
	{
		mVec128 = v128;
	}
#else
	cbtScalar m_floats[4];
#endif  // BT_USE_SSE

#endif  //__CELLOS_LV2__ __SPU__

public:
#if (defined(BT_USE_SSE_IN_API) && defined(BT_USE_SSE)) || defined(BT_USE_NEON)

	// Set Vector
	SIMD_FORCE_INLINE cbtQuadWord(const cbtSimdFloat4 vec)
	{
		mVec128 = vec;
	}

	// Copy constructor
	SIMD_FORCE_INLINE cbtQuadWord(const cbtQuadWord& rhs)
	{
		mVec128 = rhs.mVec128;
	}

	// Assignment Operator
	SIMD_FORCE_INLINE cbtQuadWord&
	operator=(const cbtQuadWord& v)
	{
		mVec128 = v.mVec128;

		return *this;
	}

#endif

	/**@brief Return the x value */
	SIMD_FORCE_INLINE const cbtScalar& getX() const { return m_floats[0]; }
	/**@brief Return the y value */
	SIMD_FORCE_INLINE const cbtScalar& getY() const { return m_floats[1]; }
	/**@brief Return the z value */
	SIMD_FORCE_INLINE const cbtScalar& getZ() const { return m_floats[2]; }
	/**@brief Set the x value */
	SIMD_FORCE_INLINE void setX(cbtScalar _x) { m_floats[0] = _x; };
	/**@brief Set the y value */
	SIMD_FORCE_INLINE void setY(cbtScalar _y) { m_floats[1] = _y; };
	/**@brief Set the z value */
	SIMD_FORCE_INLINE void setZ(cbtScalar _z) { m_floats[2] = _z; };
	/**@brief Set the w value */
	SIMD_FORCE_INLINE void setW(cbtScalar _w) { m_floats[3] = _w; };
	/**@brief Return the x value */
	SIMD_FORCE_INLINE const cbtScalar& x() const { return m_floats[0]; }
	/**@brief Return the y value */
	SIMD_FORCE_INLINE const cbtScalar& y() const { return m_floats[1]; }
	/**@brief Return the z value */
	SIMD_FORCE_INLINE const cbtScalar& z() const { return m_floats[2]; }
	/**@brief Return the w value */
	SIMD_FORCE_INLINE const cbtScalar& w() const { return m_floats[3]; }

	//SIMD_FORCE_INLINE cbtScalar&       operator[](int i)       { return (&m_floats[0])[i];	}
	//SIMD_FORCE_INLINE const cbtScalar& operator[](int i) const { return (&m_floats[0])[i]; }
	///operator cbtScalar*() replaces operator[], using implicit conversion. We added operator != and operator == to avoid pointer comparisons.
	SIMD_FORCE_INLINE operator cbtScalar*() { return &m_floats[0]; }
	SIMD_FORCE_INLINE operator const cbtScalar*() const { return &m_floats[0]; }

	SIMD_FORCE_INLINE bool operator==(const cbtQuadWord& other) const
	{
#ifdef BT_USE_SSE
		return (0xf == _mm_movemask_ps((__m128)_mm_cmpeq_ps(mVec128, other.mVec128)));
#else
		return ((m_floats[3] == other.m_floats[3]) &&
				(m_floats[2] == other.m_floats[2]) &&
				(m_floats[1] == other.m_floats[1]) &&
				(m_floats[0] == other.m_floats[0]));
#endif
	}

	SIMD_FORCE_INLINE bool operator!=(const cbtQuadWord& other) const
	{
		return !(*this == other);
	}

	/**@brief Set x,y,z and zero w 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
	SIMD_FORCE_INLINE void setValue(const cbtScalar& _x, const cbtScalar& _y, const cbtScalar& _z)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = 0.f;
	}

	/*		void getValue(cbtScalar *m) const 
		{
			m[0] = m_floats[0];
			m[1] = m_floats[1];
			m[2] = m_floats[2];
		}
*/
	/**@brief Set the values 
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
	SIMD_FORCE_INLINE void setValue(const cbtScalar& _x, const cbtScalar& _y, const cbtScalar& _z, const cbtScalar& _w)
	{
		m_floats[0] = _x;
		m_floats[1] = _y;
		m_floats[2] = _z;
		m_floats[3] = _w;
	}
	/**@brief No initialization constructor */
	SIMD_FORCE_INLINE cbtQuadWord()
	//	:m_floats[0](cbtScalar(0.)),m_floats[1](cbtScalar(0.)),m_floats[2](cbtScalar(0.)),m_floats[3](cbtScalar(0.))
	{
	}

	/**@brief Three argument constructor (zeros w)
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   */
	SIMD_FORCE_INLINE cbtQuadWord(const cbtScalar& _x, const cbtScalar& _y, const cbtScalar& _z)
	{
		m_floats[0] = _x, m_floats[1] = _y, m_floats[2] = _z, m_floats[3] = 0.0f;
	}

	/**@brief Initializing constructor
   * @param x Value of x
   * @param y Value of y
   * @param z Value of z
   * @param w Value of w
   */
	SIMD_FORCE_INLINE cbtQuadWord(const cbtScalar& _x, const cbtScalar& _y, const cbtScalar& _z, const cbtScalar& _w)
	{
		m_floats[0] = _x, m_floats[1] = _y, m_floats[2] = _z, m_floats[3] = _w;
	}

	/**@brief Set each element to the max of the current values and the values of another cbtQuadWord
   * @param other The other cbtQuadWord to compare with 
   */
	SIMD_FORCE_INLINE void setMax(const cbtQuadWord& other)
	{
#ifdef BT_USE_SSE
		mVec128 = _mm_max_ps(mVec128, other.mVec128);
#elif defined(BT_USE_NEON)
		mVec128 = vmaxq_f32(mVec128, other.mVec128);
#else
		cbtSetMax(m_floats[0], other.m_floats[0]);
		cbtSetMax(m_floats[1], other.m_floats[1]);
		cbtSetMax(m_floats[2], other.m_floats[2]);
		cbtSetMax(m_floats[3], other.m_floats[3]);
#endif
	}
	/**@brief Set each element to the min of the current values and the values of another cbtQuadWord
   * @param other The other cbtQuadWord to compare with 
   */
	SIMD_FORCE_INLINE void setMin(const cbtQuadWord& other)
	{
#ifdef BT_USE_SSE
		mVec128 = _mm_min_ps(mVec128, other.mVec128);
#elif defined(BT_USE_NEON)
		mVec128 = vminq_f32(mVec128, other.mVec128);
#else
		cbtSetMin(m_floats[0], other.m_floats[0]);
		cbtSetMin(m_floats[1], other.m_floats[1]);
		cbtSetMin(m_floats[2], other.m_floats[2]);
		cbtSetMin(m_floats[3], other.m_floats[3]);
#endif
	}
};

#endif  //BT_SIMD_QUADWORD_H
