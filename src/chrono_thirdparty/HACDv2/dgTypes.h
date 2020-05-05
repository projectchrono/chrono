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

#if !defined(AFX_DGTYPES__42YH_HY78GT_YHJ63Y__INCLUDED_)
#define AFX_DGTYPES__42YH_HY78GT_YHJ63Y__INCLUDED_

#include "PlatformConfigHACD.h"
#include <math.h>
#include <float.h>

class dgTriplex
{
	public:
	hacd::HaF32 m_x;
	hacd::HaF32 m_y;
	hacd::HaF32 m_z;
};

#define DG_MEMORY_GRANULARITY 16

#define dgPI			 	hacd::HaF32 (3.14159f)
#define dgPI2			 	hacd::HaF32 (dgPI * 2.0f)
#define dgEXP			 	hacd::HaF32 (2.71828f)
#define dgEPSILON	  	 	hacd::HaF32 (1.0e-5f)
#define dgGRAVITY	  	 	hacd::HaF32 (9.8f)
#define dgDEG2RAD	  	 	hacd::HaF32 (dgPI / 180.0f)
#define dgRAD2DEG	  	 	hacd::HaF32 (180.0f / dgPI)
#define dgKMH2MPSEC		 	hacd::HaF32 (0.278f)


class dgVector;
class dgBigVector;

#define dgApi __cdecl 	
//#ifdef _DEBUG
#ifdef _WINDOWS
#define dgCheckFloat(x) (_finite(x) && !_isnan(x))
#else
#define dgCheckFloat(x) (isfinite(x) && !isnan(x))
#endif
//#endif

HACD_INLINE hacd::HaI32 exp_2 (hacd::HaI32 x)
{
	hacd::HaI32 exp;
	for (exp = -1; x; x >>= 1) {
		exp ++;
	}
	return exp;
}

template <class T> HACD_INLINE T ClampValue(T val, T min, T max)
{
	if (val < min) {
		return min;
	}
	if (val > max) {
		return max;
	}
	return val;
}

template <class T> HACD_INLINE T GetMin(T A, T B)
{
	if (B < A) {
		A = B;
	}
	return A;
}

template <class T> HACD_INLINE T GetMax(T A, T B)
{
	if (B > A) {
		A = B;
	}
	return A;
}



template <class T> HACD_INLINE T GetMin(T A, T B, T C)
{
	return GetMin(GetMin (A, B), C);
}



template <class T> HACD_INLINE T GetMax(T A, T B, T C)
{
	return GetMax(GetMax (A, B), C);
}

template <class T> HACD_INLINE void Swap(T& A, T& B)
{
	T tmp (A);
	A = B;
	B = tmp;
}	


template <class T> HACD_INLINE T GetSign(T A)
{
	T sign (1.0f);
	if (A < T (0.0f)) {
		sign = T (-1.0f);
	}
	return sign;
}

template <class T> 
hacd::HaI32 dgBinarySearch (T const* array, hacd::HaI32 elements, hacd::HaI32 entry)
{
	hacd::HaI32 index0;
	hacd::HaI32 index1;
	hacd::HaI32 index2;
	hacd::HaI32 entry0;
	hacd::HaI32 entry1;
	hacd::HaI32 entry2;

	index0 = 0;
	index2 = elements - 1;
	entry0 = array[index0].m_Key;
	entry2 = array[index2].m_Key;

   while ((index2 - index0) > 1) {
      index1 = (index0 + index2) >> 1;
		entry1 = array[index1].m_Key;
		if (entry1 == entry) {
			HACD_ASSERT (array[index1].m_Key <= entry);
			HACD_ASSERT (array[index1 + 1].m_Key >= entry);
			return index1;
		} else if (entry < entry1) {
			index2 = index1;
			entry2 = entry1;
		} else {
			index0 = index1;
			entry0 = entry1;
		}
	}

	if (array[index0].m_Key > index0) {
		index0 --;
	}

	HACD_ASSERT (array[index0].m_Key <= entry);
	HACD_ASSERT (array[index0 + 1].m_Key >= entry);
	return index0;
}


#include <string.h> ///This fixes the memset error - Hammad

template <class T> 
void dgRadixSort (T* const array, T* const tmpArray, hacd::HaI32 elements, hacd::HaI32 radixPass, 
				  hacd::HaI32 (*getRadixKey) (const T* const  A, void* const context), void* const context = NULL)
{
	hacd::HaI32 scanCount[256]; 
	hacd::HaI32 histogram[256][4];

	HACD_ASSERT (radixPass >= 1);
	HACD_ASSERT (radixPass <= 4);
	
	memset (histogram, 0, sizeof (histogram));
	for (hacd::HaI32 i = 0; i < elements; i ++) {
		hacd::HaI32 key = getRadixKey (&array[i], context);
		for (hacd::HaI32 j = 0; j < radixPass; j ++) {
			hacd::HaI32 radix = (key >> (j << 3)) & 0xff;
			histogram[radix][j] = histogram[radix][j] + 1;
		}
	}

	for (hacd::HaI32 radix = 0; radix < radixPass; radix += 2) {
		scanCount[0] = 0;
		for (hacd::HaI32 i = 1; i < 256; i ++) {
			scanCount[i] = scanCount[i - 1] + histogram[i - 1][radix];
		}
		hacd::HaI32 radixShift = radix << 3;
		for (hacd::HaI32 i = 0; i < elements; i ++) {
			hacd::HaI32 key = (getRadixKey (&array[i], context) >> radixShift) & 0xff;
			hacd::HaI32 index = scanCount[key];
			tmpArray[index] = array[i];
			scanCount[key] = index + 1;
		}

		if ((radix + 1) < radixPass) { 
			scanCount[0] = 0;
			for (hacd::HaI32 i = 1; i < 256; i ++) {
				scanCount[i] = scanCount[i - 1] + histogram[i - 1][radix + 1];
			}
			
			hacd::HaI32 radixShift = (radix + 1) << 3;
			for (hacd::HaI32 i = 0; i < elements; i ++) {
				hacd::HaI32 key = (getRadixKey (&array[i], context) >> radixShift) & 0xff;
				hacd::HaI32 index = scanCount[key];
				array[index] = tmpArray[i];
				scanCount[key] = index + 1;
			}
		} else {
			memcpy (array, tmpArray, elements * sizeof (T)); 
		}
	}


#ifdef _DEBUG
	for (hacd::HaI32 i = 0; i < (elements - 1); i ++) {
		HACD_ASSERT (getRadixKey (&array[i], context) <= getRadixKey (&array[i + 1], context));
	}
#endif

}


template <class T> 
void dgSort (T* const array, hacd::HaI32 elements, hacd::HaI32 (*compare) (const T* const  A, const T* const B, void* const context), void* const context = NULL)
{
	hacd::HaI32 stride = 8;
	hacd::HaI32 stack[1024][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	hacd::HaI32 stackIndex = 1;
	while (stackIndex) {
		stackIndex --;
		hacd::HaI32 lo = stack[stackIndex][0];
		hacd::HaI32 hi = stack[stackIndex][1];
		if ((hi - lo) > stride) {
			hacd::HaI32 i = lo;
			hacd::HaI32 j = hi;
			T val (array[(lo + hi) >> 1]);
			do {    
				while (compare (&array[i], &val, context) < 0) i ++;
				while (compare (&array[j], &val, context) > 0) j --;

				if (i <= j)	{
					T tmp (array[i]);
					array[i] = array[j]; 
					array[j] = tmp;
					i++; 
					j--;
				}
			} while (i <= j);

			if (i < hi) {
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex ++;
			}
			if (lo < j) {
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex ++;
			}
			HACD_ASSERT (stackIndex < hacd::HaI32 (sizeof (stack) / (2 * sizeof (stack[0][0]))));
		}
	}

	stride = stride * 2;
	if (elements < stride) {
		stride = elements;
	}
	for (hacd::HaI32 i = 1; i < stride; i ++) {
		if (compare (&array[0], &array[i], context) > 0) {
			T tmp (array[0]);
			array[0] = array[i];
			array[i] = tmp;
		}
	}

	for (hacd::HaI32 i = 1; i < elements; i ++) {
		hacd::HaI32 j = i;
		T tmp (array[i]);
		//for (; j && (compare (&array[j - 1], &tmp, context) > 0); j --) {
		for (; compare (&array[j - 1], &tmp, context) > 0; j --) {
			HACD_ASSERT (j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

#ifdef _DEBUG
	for (hacd::HaI32 i = 0; i < (elements - 1); i ++) {
		HACD_ASSERT (compare (&array[i], &array[i + 1], context) <= 0);
	}
#endif
}


template <class T> 
void dgSortIndirect (T** const array, hacd::HaI32 elements, hacd::HaI32 (*compare) (const T* const  A, const T* const B, void* const context), void* const context = NULL)
{
	hacd::HaI32 stride = 8;
	hacd::HaI32 stack[1024][2];

	stack[0][0] = 0;
	stack[0][1] = elements - 1;
	hacd::HaI32 stackIndex = 1;
	while (stackIndex) {
		stackIndex --;
		hacd::HaI32 lo = stack[stackIndex][0];
		hacd::HaI32 hi = stack[stackIndex][1];
		if ((hi - lo) > stride) {
			hacd::HaI32 i = lo;
			hacd::HaI32 j = hi;
			T* val (array[(lo + hi) >> 1]);
			do {    
				while (compare (array[i], val, context) < 0) i ++;
				while (compare (array[j], val, context) > 0) j --;

				if (i <= j)	{
					T* tmp (array[i]);
					array[i] = array[j]; 
					array[j] = tmp;
					i++; 
					j--;
				}
			} while (i <= j);

			if (i < hi) {
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex ++;
			}
			if (lo < j) {
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex ++;
			}
			HACD_ASSERT (stackIndex < hacd::HaI32 (sizeof (stack) / (2 * sizeof (stack[0][0]))));
		}
	}

	stride = stride * 2;
	if (elements < stride) {
		stride = elements;
	}
	for (hacd::HaI32 i = 1; i < stride; i ++) {
		if (compare (&array[0], &array[i], context) > 0) {
			T tmp (array[0]);
			array[0] = array[i];
			array[i] = tmp;
		}
	}

	for (hacd::HaI32 i = 1; i < elements; i ++) {
		hacd::HaI32 j = i;
		T* tmp (array[i]);
		//for (; j && (compare (array[j - 1], tmp, context) > 0); j --) {
		for (; compare (&array[j - 1], &tmp, context) > 0; j --) {
			HACD_ASSERT (j > 0);
			array[j] = array[j - 1];
		}
		array[j] = tmp;
	}

#ifdef _DEBUG
	for (hacd::HaI32 i = 0; i < (elements - 1); i ++) {
		HACD_ASSERT (compare (array[i], array[i + 1], context) <= 0);
	}
#endif
}



#ifdef __USE_DOUBLE_PRECISION__
	union dgFloatSign
	{
		struct {
			hacd::HaI32 m_dommy;
			hacd::HaI32 m_iVal;
		} m_integer;
		hacd::HaF64 m_fVal;
	};
#else
	union dgFloatSign
	{
		struct {
			hacd::HaI32 m_iVal;
		} m_integer;
		hacd::HaF32 m_fVal;
	};
#endif

union dgDoubleInt
{
	struct {
		hacd::HaI32 m_intL;
		hacd::HaI32 m_intH;
	} _dgDoubleInt;
	hacd::HaI64 m_int;
	hacd::HaF64 m_float;
};



void GetMinMax (dgVector &Min, dgVector &Max, const hacd::HaF32* const vArray, hacd::HaI32 vCount, hacd::HaI32 StrideInBytes);
void GetMinMax (dgBigVector &Min, dgBigVector &Max, const hacd::HaF64* const vArray, hacd::HaI32 vCount, hacd::HaI32 strideInBytes);

hacd::HaI32 dgVertexListToIndexList (hacd::HaF32* const vertexList, hacd::HaI32 strideInBytes, hacd::HaI32 floatSizeInBytes,
				       hacd::HaI32 unsignedSizeInBytes, hacd::HaI32 vertexCount, hacd::HaI32* const indexListOut, hacd::HaF32 tolerance = dgEPSILON);

hacd::HaI32 dgVertexListToIndexList (hacd::HaF64* const vertexList, hacd::HaI32 strideInBytes, hacd::HaI32 compareCount, hacd::HaI32 vertexCount, hacd::HaI32* const indexListOut, hacd::HaF64 tolerance = dgEPSILON);



#define PointerToInt(x) ((size_t)x)
#define IntToPointer(x) ((void*)(size_t(x)))

#define dgControlFP(x,y) _controlfp(x,y)

HACD_INLINE hacd::HaF32 dgAbsf(hacd::HaF32 x)
{
	// according to Intel this is better because is doe not read after write
	return (x >= hacd::HaF32 (0.0f)) ? x : -x;
}


HACD_INLINE hacd::HaI32 dgFastInt (hacd::HaF64 x)
{
	hacd::HaI32 i = hacd::HaI32 (x);
	if (hacd::HaF64 (i) > x) {
		i --;
	}
	return i;
}

HACD_INLINE hacd::HaI32 dgFastInt (hacd::HaF32 x)
{
	hacd::HaI32 i = hacd::HaI32 (x);
	if (hacd::HaF32 (i) > x) {
		i --;
	}
	return i;
}

HACD_INLINE hacd::HaF32 dgFloor(hacd::HaF32 x)
{
#ifdef _MSC_VER
	hacd::HaF32 ret = hacd::HaF32 (dgFastInt (x));
	HACD_ASSERT (ret == floor (x));
	return  ret;
#else 
	return floor (x);
#endif
}

HACD_INLINE hacd::HaF32 dgCeil(hacd::HaF32 x)
{
#ifdef _MSC_VER
	hacd::HaF32 ret = dgFloor(x);
	if (ret < x) {
		ret += hacd::HaF32 (1.0f);
	}
	HACD_ASSERT (ret == ceil (x));
	return  ret;
#else 
	return ceil (x);
#endif
}

#define dgSqrt(x) hacd::HaF32 (sqrt(x))	
#define dgRsqrt(x) (hacd::HaF32 (1.0f) / dgSqrt(x))		
#define dgSin(x) hacd::HaF32 (sin(x))
#define dgCos(x) hacd::HaF32 (cos(x))
#define dgAsin(x) hacd::HaF32 (asin(x))
#define dgAcos(x) hacd::HaF32 (acos(x))
#define dgAtan2(x,y) hacd::HaF32 (atan2(x,y))
#define dgLog(x) hacd::HaF32 (log(x))
#define dgPow(x,y) hacd::HaF32 (pow(x,y))
#define dgFmod(x,y) hacd::HaF32 (fmod(x,y))


#endif

