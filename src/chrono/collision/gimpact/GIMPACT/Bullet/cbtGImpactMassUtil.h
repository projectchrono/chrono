/*! \file cbtGImpactMassUtil.h
\author Francisco Leon Najera
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

#ifndef GIMPACT_MASS_UTIL_H
#define GIMPACT_MASS_UTIL_H

#include "LinearMath/cbtTransform.h"

SIMD_FORCE_INLINE cbtVector3 gim_inertia_add_transformed(
	const cbtVector3& source_inertia, const cbtVector3& added_inertia, const cbtTransform& transform)
{
	cbtMatrix3x3 rotatedTensor = transform.getBasis().scaled(added_inertia) * transform.getBasis().transpose();

	cbtScalar x2 = transform.getOrigin()[0];
	x2 *= x2;
	cbtScalar y2 = transform.getOrigin()[1];
	y2 *= y2;
	cbtScalar z2 = transform.getOrigin()[2];
	z2 *= z2;

	cbtScalar ix = rotatedTensor[0][0] * (y2 + z2);
	cbtScalar iy = rotatedTensor[1][1] * (x2 + z2);
	cbtScalar iz = rotatedTensor[2][2] * (x2 + y2);

	return cbtVector3(source_inertia[0] + ix, source_inertia[1] + iy, source_inertia[2] + iz);
}

SIMD_FORCE_INLINE cbtVector3 gim_get_point_inertia(const cbtVector3& point, cbtScalar mass)
{
	cbtScalar x2 = point[0] * point[0];
	cbtScalar y2 = point[1] * point[1];
	cbtScalar z2 = point[2] * point[2];
	return cbtVector3(mass * (y2 + z2), mass * (x2 + z2), mass * (x2 + y2));
}

#endif  //GIMPACT_MESH_SHAPE_H
