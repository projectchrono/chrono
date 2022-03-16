/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_CONVEX_CAST_H
#define BT_CONVEX_CAST_H

#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtScalar.h"
class cbtMinkowskiSumShape;
#include "LinearMath/cbtIDebugDraw.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define MAX_CONVEX_CAST_ITERATIONS 64
#define MAX_CONVEX_CAST_EPSILON (SIMD_EPSILON * 10)
#else
#define MAX_CONVEX_CAST_ITERATIONS 32
#define MAX_CONVEX_CAST_EPSILON cbtScalar(0.0001)
#endif
///Typically the conservative advancement reaches solution in a few iterations, clip it to 32 for degenerate cases.
///See discussion about this here http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=565
//will need to digg deeper to make the algorithm more robust
//since, a large epsilon can cause an early termination with false
//positive results (ray intersections that shouldn't be there)

/// cbtConvexCast is an interface for Casting
class cbtConvexCast
{
public:
	virtual ~cbtConvexCast();

	///RayResult stores the closest result
	/// alternatively, add a callback method to decide about closest/all results
	struct CastResult
	{
		//virtual bool	addRayResult(const cbtVector3& normal,cbtScalar	fraction) = 0;

		virtual void DebugDraw(cbtScalar fraction) { (void)fraction; }
		virtual void drawCoordSystem(const cbtTransform& trans) { (void)trans; }
		virtual void reportFailure(int errNo, int numIterations)
		{
			(void)errNo;
			(void)numIterations;
		}
		CastResult()
			: m_fraction(cbtScalar(BT_LARGE_FLOAT)),
			  m_debugDrawer(0),
			  m_allowedPenetration(cbtScalar(0)),
			  m_subSimplexCastMaxIterations(MAX_CONVEX_CAST_ITERATIONS),
			  m_subSimplexCastEpsilon(MAX_CONVEX_CAST_EPSILON)
		{
		}

		virtual ~CastResult(){};

		cbtTransform m_hitTransformA;
		cbtTransform m_hitTransformB;
		cbtVector3 m_normal;
		cbtVector3 m_hitPoint;
		cbtScalar m_fraction;  //input and output
		cbtIDebugDraw* m_debugDrawer;
		cbtScalar m_allowedPenetration;
		
		int m_subSimplexCastMaxIterations;
		cbtScalar m_subSimplexCastEpsilon;

	};

	/// cast a convex against another convex object
	virtual bool calcTimeOfImpact(
		const cbtTransform& fromA,
		const cbtTransform& toA,
		const cbtTransform& fromB,
		const cbtTransform& toB,
		CastResult& result) = 0;
};

#endif  //BT_CONVEX_CAST_H
