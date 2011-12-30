/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __dgGoogol__
#define __dgGoogol__


#include "dgArray.h"
#include "dgVector.h"



//#define DG_GOOGOL_SIZE	16
#define DG_GOOGOL_SIZE		4

class dgGoogol
{
	public:
	dgGoogol(void);
	dgGoogol(hacd::HaF64 value);
	~dgGoogol(void);

	hacd::HaF64 GetAproximateValue() const;
	void InitFloatFloat (hacd::HaF64 value);

	dgGoogol operator+ (const dgGoogol &A) const; 
	dgGoogol operator- (const dgGoogol &A) const; 
	dgGoogol operator* (const dgGoogol &A) const; 
	dgGoogol operator/ (const dgGoogol &A) const; 

	dgGoogol operator+= (const dgGoogol &A); 
	dgGoogol operator-= (const dgGoogol &A); 

	dgGoogol Floor () const;

#ifdef _DEBUG
	void ToString (char* const string) const;
#endif

	private:
	void NegateMantissa (hacd::HaU64* const mantissa) const;
	void CopySignedMantissa (hacd::HaU64* const mantissa) const;
	hacd::HaI32 NormalizeMantissa (hacd::HaU64* const mantissa) const;
	hacd::HaU64 CheckCarrier (hacd::HaU64 a, hacd::HaU64 b) const;
	void ShiftRightMantissa (hacd::HaU64* const mantissa, hacd::HaI32 bits) const;

	hacd::HaI32 LeadinZeros (hacd::HaU64 a) const;
	void ExtendeMultiply (hacd::HaU64 a, hacd::HaU64 b, hacd::HaU64& high, hacd::HaU64& low) const;
	void ScaleMantissa (hacd::HaU64* const out, hacd::HaU64 scale) const;

	hacd::HaI8 m_sign;
	hacd::HaI16 m_exponent;
	hacd::HaU64 m_mantissa[DG_GOOGOL_SIZE];
};


class dgHugeVector: public dgTemplateVector<dgGoogol>
{
	public:
	dgHugeVector ()
		:dgTemplateVector<dgGoogol>()
	{
	}

	dgHugeVector (const dgBigVector& a)
		:dgTemplateVector<dgGoogol>(dgGoogol (a.m_x), dgGoogol (a.m_y), dgGoogol (a.m_z), dgGoogol (a.m_w))
	{
	}

	dgHugeVector (const dgTemplateVector<dgGoogol>& a)
		:dgTemplateVector<dgGoogol>(a)
	{
	}

	dgHugeVector (hacd::HaF64 x, hacd::HaF64 y, hacd::HaF64 z, hacd::HaF64 w)
		:dgTemplateVector<dgGoogol>(x, y, z, w)
	{
	}

	dgGoogol EvaluePlane (const dgHugeVector& point) const 
	{
		return (point % (*this)) + m_w;
	}




};


#endif
