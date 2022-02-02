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

#ifndef BT_DISCRETE_COLLISION_DETECTOR1_INTERFACE_H
#define BT_DISCRETE_COLLISION_DETECTOR1_INTERFACE_H

#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtVector3.h"

/// This interface is made to be used by an iterative approach to do TimeOfImpact calculations
/// This interface allows to query for closest points and penetration depth between two (convex) objects
/// the closest point is on the second object (B), and the normal points from the surface on B towards A.
/// distance is between closest points on B and closest point on A. So you can calculate closest point on A
/// by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB
struct cbtDiscreteCollisionDetectorInterface
{
	struct Result
	{
		virtual ~Result() {}

		///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
		virtual void setShapeIdentifiersA(int partId0, int index0) = 0;
		virtual void setShapeIdentifiersB(int partId1, int index1) = 0;
		virtual void addContactPoint(const cbtVector3& normalOnBInWorld, const cbtVector3& pointInWorld, cbtScalar depth) = 0;
	};

	struct ClosestPointInput
	{
		ClosestPointInput()
			: m_maximumDistanceSquared(cbtScalar(BT_LARGE_FLOAT))
		{
		}

		cbtTransform m_transformA;
		cbtTransform m_transformB;
		cbtScalar m_maximumDistanceSquared;
	};

	virtual ~cbtDiscreteCollisionDetectorInterface(){};

	//
	// give either closest points (distance > 0) or penetration (distance)
	// the normal always points from B towards A
	//
	virtual void getClosestPoints(const ClosestPointInput& input, Result& output, class cbtIDebugDraw* debugDraw, bool swapResults = false) = 0;
};

struct cbtStorageResult : public cbtDiscreteCollisionDetectorInterface::Result
{
	cbtVector3 m_normalOnSurfaceB;
	cbtVector3 m_closestPointInB;
	cbtScalar m_distance;  //negative means penetration !

protected:
	cbtStorageResult() : m_distance(cbtScalar(BT_LARGE_FLOAT))
	{
	}

public:
	virtual ~cbtStorageResult(){};

	virtual void addContactPoint(const cbtVector3& normalOnBInWorld, const cbtVector3& pointInWorld, cbtScalar depth)
	{
		if (depth < m_distance)
		{
			m_normalOnSurfaceB = normalOnBInWorld;
			m_closestPointInB = pointInWorld;
			m_distance = depth;
		}
	}
};

#endif  //BT_DISCRETE_COLLISION_DETECTOR1_INTERFACE_H
