#ifndef BT_BASIC_GEOMETRY_OPERATIONS_H_INCLUDED
#define BT_BASIC_GEOMETRY_OPERATIONS_H_INCLUDED

/*! \file cbtGeometryOperations.h
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

#include "cbtBoxCollision.h"

#define PLANEDIREPSILON 0.0000001f
#define PARALELENORMALS 0.000001f

#define BT_CLAMP(number, minval, maxval) (number < minval ? minval : (number > maxval ? maxval : number))

/// Calc a plane from a triangle edge an a normal. plane is a vec4f
SIMD_FORCE_INLINE void bt_edge_plane(const cbtVector3 &e1, const cbtVector3 &e2, const cbtVector3 &normal, cbtVector4 &plane)
{
	cbtVector3 planenormal = (e2 - e1).cross(normal);
	planenormal.normalize();
	plane.setValue(planenormal[0], planenormal[1], planenormal[2], e2.dot(planenormal));
}

//***************** SEGMENT and LINE FUNCTIONS **********************************///

/*! Finds the closest point(cp) to (v) on a segment (e1,e2)
 */
SIMD_FORCE_INLINE void bt_closest_point_on_segment(
	cbtVector3 &cp, const cbtVector3 &v,
	const cbtVector3 &e1, const cbtVector3 &e2)
{
	cbtVector3 n = e2 - e1;
	cp = v - e1;
	cbtScalar _scalar = cp.dot(n) / n.dot(n);
	if (_scalar < 0.0f)
	{
		cp = e1;
	}
	else if (_scalar > 1.0f)
	{
		cp = e2;
	}
	else
	{
		cp = _scalar * n + e1;
	}
}

//! line plane collision
/*!
*\return
	-0  if the ray never intersects
	-1 if the ray collides in front
	-2 if the ray collides in back
*/

SIMD_FORCE_INLINE int bt_line_plane_collision(
	const cbtVector4 &plane,
	const cbtVector3 &vDir,
	const cbtVector3 &vPoint,
	cbtVector3 &pout,
	cbtScalar &tparam,
	cbtScalar tmin, cbtScalar tmax)
{
	cbtScalar _dotdir = vDir.dot(plane);

	if (cbtFabs(_dotdir) < PLANEDIREPSILON)
	{
		tparam = tmax;
		return 0;
	}

	cbtScalar _dis = bt_distance_point_plane(plane, vPoint);
	char returnvalue = _dis < 0.0f ? 2 : 1;
	tparam = -_dis / _dotdir;

	if (tparam < tmin)
	{
		returnvalue = 0;
		tparam = tmin;
	}
	else if (tparam > tmax)
	{
		returnvalue = 0;
		tparam = tmax;
	}
	pout = tparam * vDir + vPoint;
	return returnvalue;
}

//! Find closest points on segments
SIMD_FORCE_INLINE void bt_segment_collision(
	const cbtVector3 &vA1,
	const cbtVector3 &vA2,
	const cbtVector3 &vB1,
	const cbtVector3 &vB2,
	cbtVector3 &vPointA,
	cbtVector3 &vPointB)
{
	cbtVector3 AD = vA2 - vA1;
	cbtVector3 BD = vB2 - vB1;
	cbtVector3 N = AD.cross(BD);
	cbtScalar tp = N.length2();

	cbtVector4 _M;  //plane

	if (tp < SIMD_EPSILON)  //ARE PARALELE
	{
		//project B over A
		bool invert_b_order = false;
		_M[0] = vB1.dot(AD);
		_M[1] = vB2.dot(AD);

		if (_M[0] > _M[1])
		{
			invert_b_order = true;
			BT_SWAP_NUMBERS(_M[0], _M[1]);
		}
		_M[2] = vA1.dot(AD);
		_M[3] = vA2.dot(AD);
		//mid points
		N[0] = (_M[0] + _M[1]) * 0.5f;
		N[1] = (_M[2] + _M[3]) * 0.5f;

		if (N[0] < N[1])
		{
			if (_M[1] < _M[2])
			{
				vPointB = invert_b_order ? vB1 : vB2;
				vPointA = vA1;
			}
			else if (_M[1] < _M[3])
			{
				vPointB = invert_b_order ? vB1 : vB2;
				bt_closest_point_on_segment(vPointA, vPointB, vA1, vA2);
			}
			else
			{
				vPointA = vA2;
				bt_closest_point_on_segment(vPointB, vPointA, vB1, vB2);
			}
		}
		else
		{
			if (_M[3] < _M[0])
			{
				vPointB = invert_b_order ? vB2 : vB1;
				vPointA = vA2;
			}
			else if (_M[3] < _M[1])
			{
				vPointA = vA2;
				bt_closest_point_on_segment(vPointB, vPointA, vB1, vB2);
			}
			else
			{
				vPointB = invert_b_order ? vB1 : vB2;
				bt_closest_point_on_segment(vPointA, vPointB, vA1, vA2);
			}
		}
		return;
	}

	N = N.cross(BD);
	_M.setValue(N[0], N[1], N[2], vB1.dot(N));

	// get point A as the plane collision point
	bt_line_plane_collision(_M, AD, vA1, vPointA, tp, cbtScalar(0), cbtScalar(1));

	/*Closest point on segment*/
	vPointB = vPointA - vB1;
	tp = vPointB.dot(BD);
	tp /= BD.dot(BD);
	tp = BT_CLAMP(tp, 0.0f, 1.0f);

	vPointB = tp * BD + vB1;
}

#endif  // GIM_VECTOR_H_INCLUDED
