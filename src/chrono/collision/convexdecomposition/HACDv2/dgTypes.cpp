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

#include "dgTypes.h"
#include "dgVector.h"
#include "dgStack.h"
#include <string.h>


void GetMinMax (dgVector &minOut, dgVector &maxOut, const hacd::HaF32* const vertexArray, hacd::HaI32 vCount, hacd::HaI32 strideInBytes)
{
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF32));
	const hacd::HaF32* vArray = vertexArray + stride;

	HACD_ASSERT (stride >= 3);
 	minOut = dgVector (vertexArray[0], vertexArray[1], vertexArray[2], hacd::HaF32 (0.0f)); 
	maxOut = dgVector (vertexArray[0], vertexArray[1], vertexArray[2], hacd::HaF32 (0.0f)); 

	for (hacd::HaI32 i = 1; i < vCount; i ++) {
		minOut.m_x = GetMin (minOut.m_x, vArray[0]);
		minOut.m_y = GetMin (minOut.m_y, vArray[1]);
		minOut.m_z = GetMin (minOut.m_z, vArray[2]);

		maxOut.m_x = GetMax (maxOut.m_x, vArray[0]);
		maxOut.m_y = GetMax (maxOut.m_y, vArray[1]);
		maxOut.m_z = GetMax (maxOut.m_z, vArray[2]);

		vArray += stride;
	}
}


void GetMinMax (dgBigVector &minOut, dgBigVector &maxOut, const hacd::HaF64* const vertexArray, hacd::HaI32 vCount, hacd::HaI32 strideInBytes)
{
	hacd::HaI32 stride = hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64));
	const hacd::HaF64* vArray = vertexArray + stride;

	HACD_ASSERT (stride >= 3);
	minOut = dgBigVector (vertexArray[0], vertexArray[1], vertexArray[2], hacd::HaF64 (0.0f)); 
	maxOut = dgBigVector (vertexArray[0], vertexArray[1], vertexArray[2], hacd::HaF64 (0.0f)); 

	for (hacd::HaI32 i = 1; i < vCount; i ++) {
		minOut.m_x = GetMin (minOut.m_x, vArray[0]);
		minOut.m_y = GetMin (minOut.m_y, vArray[1]);
		minOut.m_z = GetMin (minOut.m_z, vArray[2]);

		maxOut.m_x = GetMax (maxOut.m_x, vArray[0]);
		maxOut.m_y = GetMax (maxOut.m_y, vArray[1]);
		maxOut.m_z = GetMax (maxOut.m_z, vArray[2]);

		vArray += stride;
	}
}



static inline hacd::HaI32 cmp_vertex (const hacd::HaF64* const v1, const hacd::HaF64* const v2, hacd::HaI32 firstSortAxis)
	{
		if (v1[firstSortAxis] < v2[firstSortAxis]) {
			return -1;
		}

		if (v1[firstSortAxis] > v2[firstSortAxis]){
			return 1;
		}

		return 0;
	}
	
static hacd::HaI32 SortVertices (hacd::HaF64* const vertexList,  hacd::HaI32 stride, hacd::HaI32 compareCount, hacd::HaI32 vertexCount, hacd::HaF64 tolerance)
	{
		hacd::HaF64 xc = 0;
		hacd::HaF64 yc = 0;
		hacd::HaF64 zc = 0;
		hacd::HaF64 x2c = 0;
		hacd::HaF64 y2c = 0;
		hacd::HaF64 z2c = 0;

		dgBigVector minP (1e10, 1e10, 1e10, 0);
		dgBigVector maxP (-1e10, -1e10, -1e10, 0);
		hacd::HaI32 k = 0;
		for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
			hacd::HaF64 x  = vertexList[k + 2];
			hacd::HaF64 y  = vertexList[k + 3];
			hacd::HaF64 z  = vertexList[k + 4];
			k += stride;

			xc += x;
			yc += y;
			zc += z;
			x2c += x * x;
			y2c += y * y; 
			z2c += z * z;
	
			if (x < minP.m_x) {
				minP.m_x = x; 
			}
			if (y < minP.m_y) {
				minP.m_y = y; 
			}
	
			if (z < minP.m_z) {
				minP.m_z = z; 
			}
	
			if (x > maxP.m_x) {
				maxP.m_x = x; 
			}
			if (y > maxP.m_y) {
				maxP.m_y = y; 
			}
	
			if (z > maxP.m_z) {
				maxP.m_z = z; 
			}
		}
	
		dgBigVector del (maxP - minP);
		hacd::HaF64 minDist = GetMin (del.m_x, del.m_y, del.m_z);
		if (minDist < 1.0e-3) {
			minDist = 1.0e-3;
		}
	
	hacd::HaF64 tol = tolerance * minDist + 1.0e-12f;
		hacd::HaF64 sweptWindow = 2.0 * tol;
		sweptWindow += 1.0e-4;
	
		x2c = vertexCount * x2c - xc * xc;
		y2c = vertexCount * y2c - yc * yc;
		z2c = vertexCount * z2c - zc * zc;

		hacd::HaI32 firstSortAxis = 2;
		if ((y2c >= x2c) && (y2c >= z2c)) {
			firstSortAxis = 3;
		} else if ((z2c >= x2c) && (z2c >= y2c)) {
			firstSortAxis = 4;
		}


		hacd::HaI32 stack[1024][2];
		stack[0][0] = 0;
		stack[0][1] = vertexCount - 1;
		hacd::HaI32 stackIndex = 1;
		while (stackIndex) {
			stackIndex --;
			hacd::HaI32 lo = stack[stackIndex][0];
			hacd::HaI32 hi = stack[stackIndex][1];
			if ((hi - lo) > 8) {
				hacd::HaI32 i = lo;
				hacd::HaI32 j = hi;
			hacd::HaF64 val[64]; 
			memcpy (val, &vertexList[((lo + hi) >> 1) * stride], stride * sizeof (hacd::HaF64));
				do {    
					while (cmp_vertex (&vertexList[i * stride], val, firstSortAxis) < 0) i ++;
					while (cmp_vertex (&vertexList[j * stride], val, firstSortAxis) > 0) j --;

					if (i <= j)	{
					hacd::HaF64 tmp[64]; 
					memcpy (tmp, &vertexList[i * stride], stride * sizeof (hacd::HaF64));
					memcpy (&vertexList[i * stride], &vertexList[j * stride], stride * sizeof (hacd::HaF64)); 
					memcpy (&vertexList[j * stride], tmp, stride * sizeof (hacd::HaF64)); 
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
			} else {
				for (hacd::HaI32 i = lo + 1; i <= hi ; i++) {
				hacd::HaF64 tmp[64]; 
				memcpy (tmp, &vertexList[i * stride], stride * sizeof (hacd::HaF64));

				hacd::HaI32 j = i;
				for (; j && (cmp_vertex (&vertexList[(j - 1) * stride], tmp, firstSortAxis) > 0); j --) {
					memcpy (&vertexList[j * stride], &vertexList[(j - 1)* stride], stride * sizeof (hacd::HaF64));
					}
				memcpy (&vertexList[j * stride], tmp, stride * sizeof (hacd::HaF64)); 
				}
			}
		}


#ifdef _DEBUG
		for (hacd::HaI32 i = 0; i < (vertexCount - 1); i ++) {
			HACD_ASSERT (cmp_vertex (&vertexList[i * stride], &vertexList[(i + 1) * stride], firstSortAxis) <= 0);
		}
#endif

		hacd::HaI32 count = 0;
		for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
			hacd::HaI32 m = i * stride;
		hacd::HaI32 index = hacd::HaI32 (vertexList[m + 0]);
			if (index == hacd::HaI32 (0xffffffff)) {
				hacd::HaF64 swept = vertexList[m + firstSortAxis] + sweptWindow;
				hacd::HaI32 k = i * stride + stride;
				for (hacd::HaI32 i1 = i + 1; i1 < vertexCount; i1 ++) {

				index = hacd::HaI32 (vertexList[k + 0]);
					if (index == hacd::HaI32 (0xffffffff)) {
						hacd::HaF64 val = vertexList[k + firstSortAxis];
						if (val >= swept) {
							break;
						}
						bool test = true;
					for (hacd::HaI32 t = 0; test && (t < compareCount); t ++) {
						hacd::HaF64 val = fabs (vertexList[m + t + 2] - vertexList[k + t + 2]);
							test = test && (val <= tol);
						}
						if (test) {
						vertexList[k + 0] = hacd::HaF64 (count);
						}
					}
					k += stride;
				}

			memcpy (&vertexList[count * stride + 2], &vertexList[m + 2], (stride - 2) * sizeof (hacd::HaF64));
			vertexList[m + 0] = hacd::HaF64 (count);
				count ++;
			}
		}
				
		return count;
	}



//static hacd::HaI32 QuickSortVertices (hacd::HaF32* const vertList, hacd::HaI32 stride, hacd::HaI32 floatSize, hacd::HaI32 unsignedSize, hacd::HaI32 vertexCount, hacd::HaF32 tolerance)
static hacd::HaI32 QuickSortVertices (hacd::HaF64* const vertList, hacd::HaI32 stride, hacd::HaI32 compareCount, hacd::HaI32 vertexCount, hacd::HaF64 tolerance)
	{
		hacd::HaI32 count = 0;
		if (vertexCount > (3 * 1024 * 32)) {
		hacd::HaF64 x = hacd::HaF32 (0.0f);
		hacd::HaF64 y = hacd::HaF32 (0.0f);
		hacd::HaF64 z = hacd::HaF32 (0.0f);
		hacd::HaF64 xd = hacd::HaF32 (0.0f);
		hacd::HaF64 yd = hacd::HaF32 (0.0f);
		hacd::HaF64 zd = hacd::HaF32 (0.0f);
			
			for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
			hacd::HaF64 x0 = vertList[i * stride + 2];
			hacd::HaF64 y0 = vertList[i * stride + 3];
			hacd::HaF64 z0 = vertList[i * stride + 4];
				x += x0;
				y += y0;
				z += z0;
				xd += x0 * x0;
				yd += y0 * y0;
				zd += z0 * z0;
			}

			xd = vertexCount * xd - x * x;
			yd = vertexCount * yd - y * y;
			zd = vertexCount * zd - z * z;

			hacd::HaI32 axis = 2;
		        hacd::HaF64 axisVal = x / vertexCount;
			if ((yd > xd) && (yd > zd)) {
				axis = 3;
				axisVal = y / vertexCount;
			}
			if ((zd > xd) && (zd > yd)) {
				axis = 4;
				axisVal = z / vertexCount;
			}

			hacd::HaI32 i0 = 0;
			hacd::HaI32 i1 = vertexCount - 1;
			do {    
				for ( ;vertList[i0 * stride + axis] < axisVal; i0 ++); 
				for ( ;vertList[i1 * stride + axis] > axisVal; i1 --);
				if (i0 <= i1) {
					for (hacd::HaI32 i = 0; i < stride; i ++) {
						Swap (vertList[i0 * stride + i], vertList[i1 * stride + i]);
					}
					i0 ++; 
					i1 --;
				}
			} while (i0 <= i1);
			HACD_ASSERT (i0 < vertexCount);

		hacd::HaI32 count0 = QuickSortVertices (&vertList[ 0 * stride], stride, compareCount, i0, tolerance);
		hacd::HaI32 count1 = QuickSortVertices (&vertList[i0 * stride], stride, compareCount, vertexCount - i0, tolerance);
			
			count = count0 + count1;

			for (hacd::HaI32 i = 0; i < count1; i ++) {
			memcpy (&vertList[(count0 + i) * stride + 2], &vertList[(i0 + i) * stride + 2], (stride - 2) * sizeof (hacd::HaF64));
			}


//		hacd::HaF64* const indexPtr = (hacd::HaI64*)vertList;
			for (hacd::HaI32 i = i0; i < vertexCount; i ++) {
//			indexPtr[i * stride] += count0;
			vertList[i * stride] += hacd::HaF64 (count0);
			}

		} else {
		count = SortVertices (vertList, stride, compareCount, vertexCount, tolerance);
		}

		return count;
	}





hacd::HaI32 dgVertexListToIndexList (hacd::HaF64* const vertList, hacd::HaI32 strideInBytes, hacd::HaI32 compareCount, hacd::HaI32 vertexCount, hacd::HaI32* const indexListOut, hacd::HaF64 tolerance)
{
#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	hacd::HaU32 controlWorld = dgControlFP (0xffffffff, 0);
	dgControlFP (_PC_53, _MCW_PC);
#endif

	if (strideInBytes < 3 * hacd::HaI32 (sizeof (hacd::HaF64))) {
		return 0;
	}
	if (compareCount < 3) {
		return 0;
	}
	HACD_ASSERT (compareCount <= hacd::HaI32 (strideInBytes / sizeof (hacd::HaF64)));
	HACD_ASSERT (strideInBytes == hacd::HaI32 (sizeof (hacd::HaF64) * (strideInBytes / sizeof (hacd::HaF64))));

	hacd::HaI32 stride = strideInBytes / hacd::HaI32 (sizeof (hacd::HaF64));
	hacd::HaI32 stride2 = stride + 2;

	dgStack<hacd::HaF64>pool (stride2  * vertexCount);
	hacd::HaF64* const tmpVertexList = &pool[0];

//	hacd::HaI64* const indexPtr = (hacd::HaI64*)tmpVertexList;

	hacd::HaI32 k = 0;
	hacd::HaI32 m = 0;
	for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
		memcpy (&tmpVertexList[m + 2], &vertList[k], stride * sizeof (hacd::HaF64));
		tmpVertexList[m + 0] = hacd::HaF64 (- 1.0f);
		tmpVertexList[m + 1] = hacd::HaF64 (i);
		k += stride;
		m += stride2;
	}
	
	hacd::HaI32 count = QuickSortVertices (tmpVertexList, stride2, compareCount, vertexCount, tolerance);

	k = 0;
	m = 0;
	for (hacd::HaI32 i = 0; i < count; i ++) {
		k = i * stride;
		m = i * stride2;
		memcpy (&vertList[k], &tmpVertexList[m + 2], stride * sizeof (hacd::HaF64));
		k += stride;
		m += stride2;
	}

	m = 0;
	for (hacd::HaI32 i = 0; i < vertexCount; i ++) {
		hacd::HaI32 i1 = hacd::HaI32 (tmpVertexList [m + 1]);
		hacd::HaI32 index = hacd::HaI32 (tmpVertexList [m + 0]);
		indexListOut[i1] = index;
		m += stride2;
	}

#if (defined (_WIN_32_VER) || defined (_WIN_64_VER))
	dgControlFP (controlWorld, _MCW_PC);
#endif

	return count;
}




hacd::HaI32 dgVertexListToIndexList (hacd::HaF32* const vertList, hacd::HaI32 strideInBytes, hacd::HaI32 floatSizeInBytes, hacd::HaI32 unsignedSizeInBytes, hacd::HaI32 vertexCount, hacd::HaI32* const indexList, hacd::HaF32 tolerance)
{
	HACD_FORCE_PARAMETER_REFERENCE(unsignedSizeInBytes);
	hacd::HaI32 stride = strideInBytes / sizeof (hacd::HaF32);

	HACD_ASSERT (!unsignedSizeInBytes);
	dgStack<hacd::HaF64> pool(vertexCount * stride);

	hacd::HaI32 floatCount = floatSizeInBytes / sizeof (hacd::HaF32);

	hacd::HaF64* const data = &pool[0];
	for (hacd::HaI32 i = 0; i < vertexCount; i ++) {

		hacd::HaF64* const dst = &data[i * stride];
		hacd::HaF32* const src = &vertList[i * stride];
		for (hacd::HaI32 j = 0; j < stride; j ++) {
			dst[j] = src[j];
		}
	}

	hacd::HaI32 count = dgVertexListToIndexList (data, stride * sizeof (hacd::HaF64), floatCount, vertexCount, indexList, hacd::HaF64 (tolerance));
	for (hacd::HaI32 i = 0; i < count; i ++) {
		hacd::HaF64* const src = &data[i * stride];
		hacd::HaF32* const dst = &vertList[i * stride];
		for (hacd::HaI32 j = 0; j < stride; j ++) {
			dst[j] = hacd::HaF32 (src[j]);
		}
	}
	
	return count;
}