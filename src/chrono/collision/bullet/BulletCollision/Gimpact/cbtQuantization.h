#ifndef BT_GIMPACT_QUANTIZATION_H_INCLUDED
#define BT_GIMPACT_QUANTIZATION_H_INCLUDED

/*! \file cbtQuantization.h
*\author Francisco Leon Najera

*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "LinearMath/cbtTransform.h"

SIMD_FORCE_INLINE void bt_calc_quantization_parameters(
	cbtVector3& outMinBound,
	cbtVector3& outMaxBound,
	cbtVector3& bvhQuantization,
	const cbtVector3& srcMinBound, const cbtVector3& srcMaxBound,
	cbtScalar quantizationMargin)
{
	//enlarge the AABB to avoid division by zero when initializing the quantization values
	cbtVector3 clampValue(quantizationMargin, quantizationMargin, quantizationMargin);
	outMinBound = srcMinBound - clampValue;
	outMaxBound = srcMaxBound + clampValue;
	cbtVector3 aabbSize = outMaxBound - outMinBound;
	bvhQuantization = cbtVector3(cbtScalar(65535.0),
								cbtScalar(65535.0),
								cbtScalar(65535.0)) /
					  aabbSize;
}

SIMD_FORCE_INLINE void bt_quantize_clamp(
	unsigned short* out,
	const cbtVector3& point,
	const cbtVector3& min_bound,
	const cbtVector3& max_bound,
	const cbtVector3& bvhQuantization)
{
	cbtVector3 clampedPoint(point);
	clampedPoint.setMax(min_bound);
	clampedPoint.setMin(max_bound);

	cbtVector3 v = (clampedPoint - min_bound) * bvhQuantization;
	out[0] = (unsigned short)(v.getX() + 0.5f);
	out[1] = (unsigned short)(v.getY() + 0.5f);
	out[2] = (unsigned short)(v.getZ() + 0.5f);
}

SIMD_FORCE_INLINE cbtVector3 bt_unquantize(
	const unsigned short* vecIn,
	const cbtVector3& offset,
	const cbtVector3& bvhQuantization)
{
	cbtVector3 vecOut;
	vecOut.setValue(
		(cbtScalar)(vecIn[0]) / (bvhQuantization.getX()),
		(cbtScalar)(vecIn[1]) / (bvhQuantization.getY()),
		(cbtScalar)(vecIn[2]) / (bvhQuantization.getZ()));
	vecOut += offset;
	return vecOut;
}

#endif  // BT_GIMPACT_QUANTIZATION_H_INCLUDED
