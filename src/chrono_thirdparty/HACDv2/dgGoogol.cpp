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


#include "dgGoogol.h"
#include <string.h>


dgGoogol::dgGoogol(void)
	:m_sign(0)
	,m_exponent(0)
{
	memset (m_mantissa, 0, sizeof (m_mantissa));
}

dgGoogol::dgGoogol(hacd::HaF64 value)
	:m_sign(0)
	,m_exponent(0)
{
	hacd::HaI32 exp;
	hacd::HaF64 mantissa = fabs (frexp(value, &exp));

	m_exponent = hacd::HaI16 (exp);
	m_sign = (value >= 0) ? 0 : 1;

	memset (m_mantissa, 0, sizeof (m_mantissa));
	m_mantissa[0] = (hacd::HaI64 (hacd::HaF64 (hacd::HaU64(1)<<62) * mantissa));

	// it looks like GCC have problems with this
//	HACD_ASSERT (m_mantissa[0] >= 0);
}


dgGoogol::~dgGoogol(void)
{
}

hacd::HaF64 dgGoogol::GetAproximateValue() const
{
	hacd::HaF64 mantissa = (hacd::HaF64(1.0f) / hacd::HaF64 (hacd::HaU64(1)<<62)) * hacd::HaF64 (m_mantissa[0]);
	mantissa = ldexp(mantissa, m_exponent) * (m_sign ?  hacd::HaF64 (-1.0f) : hacd::HaF64 (1.0f));
	return mantissa;
}

void dgGoogol::NegateMantissa (hacd::HaU64* const mantissa) const
{
	hacd::HaU64 carrier = 1;
	for (hacd::HaI32 i = DG_GOOGOL_SIZE - 1; i >= 0; i --) {
		hacd::HaU64 a = ~mantissa[i] + carrier;
		if (a) {
			carrier = 0;
		}
		mantissa[i] = a;
	}
}

void dgGoogol::CopySignedMantissa (hacd::HaU64* const mantissa) const
{
	memcpy (mantissa, m_mantissa, sizeof (m_mantissa));
	if (m_sign) {
		NegateMantissa (mantissa);
	}
}

void dgGoogol::ShiftRightMantissa (hacd::HaU64* const mantissa, hacd::HaI32 bits) const
{
	hacd::HaU64 carrier = 0;
	if (hacd::HaI64 (mantissa[0]) < hacd::HaI64 (0)) {
		carrier = hacd::HaU64 (-1);
	}
	
	while (bits >= 64) {
		for (hacd::HaI32 i = DG_GOOGOL_SIZE - 2; i >= 0; i --) {
			mantissa[i + 1] = mantissa[i];
		}
		mantissa[0] = carrier;
		bits -= 64;
	}

	if (bits > 0) {
		carrier <<= (64 - bits);
		for (hacd::HaI32 i = 0; i < DG_GOOGOL_SIZE; i ++) {
			hacd::HaU64 a = mantissa[i];
			mantissa[i] = (a >> bits) | carrier;
			carrier = a << (64 - bits);
		}
	}
}

hacd::HaI32 dgGoogol::LeadinZeros (hacd::HaU64 a) const
{
	#define dgCOUNTBIT(mask,add)		\
	{									\
		hacd::HaU64 test = a & mask;	\
		n += test ? 0 : add;			\
		a = test ? test : (a & ~mask);	\
	}

	hacd::HaI32 n = 0;
	dgCOUNTBIT (0xffffffff00000000LL, 32);
	dgCOUNTBIT (0xffff0000ffff0000LL, 16);
	dgCOUNTBIT (0xff00ff00ff00ff00LL,  8);
	dgCOUNTBIT (0xf0f0f0f0f0f0f0f0LL,  4);
	dgCOUNTBIT (0xccccccccccccccccLL,  2);
	dgCOUNTBIT (0xaaaaaaaaaaaaaaaaLL,  1);

	return n;
}

hacd::HaI32 dgGoogol::NormalizeMantissa (hacd::HaU64* const mantissa) const
{
	HACD_ASSERT (hacd::HaI64 (mantissa[0]) >= 0);

	hacd::HaI32 bits = 0;
	if(hacd::HaI64 (mantissa[0] * 2) < 0) {
		bits = 1;
		ShiftRightMantissa (mantissa, 1);
	} else {
		while (!mantissa[0] && bits > (-64 * DG_GOOGOL_SIZE)) {
			bits -= 64;
			for (hacd::HaI32 i = 1; i < DG_GOOGOL_SIZE; i ++) {
				mantissa[i - 1] = mantissa[i];
			}
			mantissa[DG_GOOGOL_SIZE - 1] = 0;
		}

		if (bits > (-64 * DG_GOOGOL_SIZE)) {
			hacd::HaI32 n = LeadinZeros (mantissa[0]) - 2;
			hacd::HaU64 carrier = 0;
			for (hacd::HaI32 i = DG_GOOGOL_SIZE-1; i >= 0; i --) {
				hacd::HaU64 a = mantissa[i];
				mantissa[i] = (a << n) | carrier;
				carrier = a >> (64 - n);
			}
			bits -= n;
		}
	}
	return bits;
}

hacd::HaU64 dgGoogol::CheckCarrier (hacd::HaU64 a, hacd::HaU64 b) const
{
	return ((hacd::HaU64 (-1) - b) < a) ? 1 : 0;
}

dgGoogol dgGoogol::operator+ (const dgGoogol &A) const
{
	dgGoogol tmp;
	if (m_mantissa[0] && A.m_mantissa[0]) {
		hacd::HaU64 mantissa0[DG_GOOGOL_SIZE];
		hacd::HaU64 mantissa1[DG_GOOGOL_SIZE];
		hacd::HaU64 mantissa[DG_GOOGOL_SIZE];

		CopySignedMantissa (mantissa0);
		A.CopySignedMantissa (mantissa1);

		hacd::HaI32 exponetDiff = m_exponent - A.m_exponent;
		hacd::HaI32 exponent = m_exponent;
		if (exponetDiff > 0) {
			ShiftRightMantissa (mantissa1, exponetDiff);
		} else if (exponetDiff < 0) {
			exponent = A.m_exponent;
			ShiftRightMantissa (mantissa0, -exponetDiff);
		} 

		hacd::HaU64 carrier = 0;
		for (hacd::HaI32 i = DG_GOOGOL_SIZE - 1; i >= 0; i --) {
			hacd::HaU64 m0 = mantissa0[i];
			hacd::HaU64 m1 = mantissa1[i];
			mantissa[i] = m0 + m1 + carrier;
			carrier = CheckCarrier (m0, m1) | CheckCarrier (m0 + m1, carrier);
		}

		hacd::HaI8 sign = 0;
		if (hacd::HaI64 (mantissa[0]) < 0) {
			sign = 1;
			NegateMantissa (mantissa);
		}

		hacd::HaI32 bits = NormalizeMantissa (mantissa);
		if (bits <= (-64 * DG_GOOGOL_SIZE)) {
			tmp.m_sign = 0;
			tmp.m_exponent = 0;
		} else {
			tmp.m_sign = sign;
			tmp.m_exponent =  hacd::HaI16 (exponent + bits);
		}

		memcpy (tmp.m_mantissa, mantissa, sizeof (m_mantissa));
		

	} else if (A.m_mantissa[0]) {
		tmp = A;
	} else {
		tmp = *this;
	}

	return tmp;
}


dgGoogol dgGoogol::operator- (const dgGoogol &A) const
{
	dgGoogol tmp (A);
	tmp.m_sign = !tmp.m_sign;
	return *this + tmp;
}


void dgGoogol::ExtendeMultiply (hacd::HaU64 a, hacd::HaU64 b, hacd::HaU64& high, hacd::HaU64& low) const
{
	hacd::HaU64 bLow = b & 0xffffffff; 
	hacd::HaU64 bHigh = b >> 32; 
	hacd::HaU64 aLow = a & 0xffffffff; 
	hacd::HaU64 aHigh = a >> 32; 

	hacd::HaU64 l = bLow * aLow;

	hacd::HaU64 c1 = bHigh * aLow;
	hacd::HaU64 c2 = bLow * aHigh;
	hacd::HaU64 m = c1 + c2;
	hacd::HaU64 carrier = CheckCarrier (c1, c2) << 32;

	hacd::HaU64 h = bHigh * aHigh + carrier;

	hacd::HaU64 ml = m << 32;	
	hacd::HaU64 ll = l + ml;
	hacd::HaU64 mh = (m >> 32) + CheckCarrier (l, ml);	
	HACD_ASSERT ((mh & ~0xffffffff) == 0);

	hacd::HaU64 hh = h + mh;

	low = ll;
	high = hh;
}

void dgGoogol::ScaleMantissa (hacd::HaU64* const dst, hacd::HaU64 scale) const
{
	hacd::HaU64 carrier = 0;
	for (hacd::HaI32 i = DG_GOOGOL_SIZE - 1; i >= 0; i --) {
		if (m_mantissa[i]) {
			hacd::HaU64 low;
			hacd::HaU64 high;
			ExtendeMultiply (scale, m_mantissa[i], high, low);
			hacd::HaU64 acc = low + carrier;
			carrier = CheckCarrier (low, carrier);	
			HACD_ASSERT (CheckCarrier (carrier, high) == 0);
			carrier += high;
			dst[i + 1] = acc;
		} else {
			dst[i + 1] = carrier;
			carrier = 0;
		}

	}
	dst[0] = carrier;
}

dgGoogol dgGoogol::operator* (const dgGoogol &A) const
{
	HACD_ASSERT (hacd::HaI64 (m_mantissa[0]) >= 0);
	HACD_ASSERT (hacd::HaI64 (A.m_mantissa[0]) >= 0);

	if (m_mantissa[0] && A.m_mantissa[0]) {
		hacd::HaU64 mantissaAcc[DG_GOOGOL_SIZE * 2];
		memset (mantissaAcc, 0, sizeof (mantissaAcc));
		for (hacd::HaI32 i = DG_GOOGOL_SIZE - 1; i >= 0; i --) {
			hacd::HaU64 a = m_mantissa[i];
			if (a) {
				hacd::HaU64 mantissaScale[2 * DG_GOOGOL_SIZE];
				memset (mantissaScale, 0, sizeof (mantissaScale));
				A.ScaleMantissa (&mantissaScale[i], a);

				hacd::HaU64 carrier = 0;
				for (hacd::HaI32 j = 2 * DG_GOOGOL_SIZE - 1; j >= 0; j --) {
					hacd::HaU64 m0 = mantissaAcc[j];
					hacd::HaU64 m1 = mantissaScale[j];
					mantissaAcc[j] = m0 + m1 + carrier;
					carrier = CheckCarrier (m0, m1) | CheckCarrier (m0 + m1, carrier);
				}
			}
		}
		
		hacd::HaU64 carrier = 0;
		hacd::HaI32 bits = LeadinZeros (mantissaAcc[0]) - 2;
		for (hacd::HaI32 i = 2 * DG_GOOGOL_SIZE - 1; i >= 0; i --) {
			hacd::HaU64 a = mantissaAcc[i];
			mantissaAcc[i] = (a << bits) | carrier;
			carrier = a >> (64 - bits);
		}

		hacd::HaI32 exp = m_exponent + A.m_exponent - (bits - 2);

		dgGoogol tmp;
		tmp.m_sign = m_sign ^ A.m_sign;
		tmp.m_exponent = hacd::HaI16 (exp);
		memcpy (tmp.m_mantissa, mantissaAcc, sizeof (m_mantissa));

		return tmp;
	} 
	return dgGoogol(0.0);
}

dgGoogol dgGoogol::operator/ (const dgGoogol &A) const
{
	dgGoogol tmp (1.0 / A.GetAproximateValue());
	dgGoogol two (2.0);
	
	tmp = tmp * (two - A * tmp);
	tmp = tmp * (two - A * tmp);
	
	hacd::HaU64 copy[DG_GOOGOL_SIZE];

	bool test = true;
	int passes = 0;
	do {
		passes ++;
		memcpy (copy, tmp.m_mantissa, sizeof (tmp.m_mantissa));
		tmp = tmp * (two - A * tmp);
		test = true;
		for (hacd::HaI32 i = 0; test && (i < DG_GOOGOL_SIZE); i ++) {
			test = (copy[i] == tmp.m_mantissa[i]);
		}
	} while (!test || (passes > (2 * DG_GOOGOL_SIZE)));	
	HACD_ASSERT (passes <= (2 * DG_GOOGOL_SIZE));
	return (*this) * tmp;
}


dgGoogol dgGoogol::operator+= (const dgGoogol &A)
{
	*this = *this + A;
	return *this;
}

dgGoogol dgGoogol::operator-= (const dgGoogol &A)
{
	*this = *this - A;
	return *this;
}

dgGoogol dgGoogol::Floor () const
{
	if (m_exponent < 1) {
		return dgGoogol (0.0);
	} 
	hacd::HaI32 bits = m_exponent + 2;
	hacd::HaI32 start = 0;
	while (bits >= 64) {
		bits -= 64;
		start ++;
	}
	
	dgGoogol tmp (*this);
	for (hacd::HaI32 i = DG_GOOGOL_SIZE - 1; i > start; i --) {
		tmp.m_mantissa[i] = 0;
	}
	hacd::HaU64 mask = (-1LL) << (64 - bits);
	tmp.m_mantissa[start] &= mask;
	if (m_sign) {
		HACD_ASSERT (0);
	}

	return tmp;
}

#ifdef _DEBUG
void dgGoogol::ToString (char* const string) const
{
	dgGoogol tmp (*this);
	dgGoogol base (10.0);

//char aaaa[256];
hacd::HaF64 a = tmp.GetAproximateValue(); 

	while (tmp.GetAproximateValue() > 1.0) {
		tmp = tmp/base;
	}
a = tmp.GetAproximateValue(); 

	hacd::HaI32 index = 0;
//	tmp.m_exponent = 1;
	while (tmp.m_mantissa[0]) {
		
//hacd::HaF64 xxx = tmp.GetAproximateValue();
//hacd::HaF64 xxx1 = digit.GetAproximateValue();
//hacd::HaF64 m = floor (a);
//a = a - m;
//a = a * 10;
//aaaa[index] = char (m) + '0';

		tmp = tmp * base;
		dgGoogol digit (tmp.Floor());
		tmp -= digit;
		hacd::HaF64 val = digit.GetAproximateValue();
		string[index] = char (val) + '0';
		index ++;
	}
	string[index] = 0;
}


#endif