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

#ifndef BT_GJK_PAIR_DETECTOR_H
#define BT_GJK_PAIR_DETECTOR_H

#include "cbtDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/CollisionShapes/cbtCollisionMargin.h"

class cbtConvexShape;
#include "cbtSimplexSolverInterface.h"
class cbtConvexPenetrationDepthSolver;

/// cbtGjkPairDetector uses GJK to implement the cbtDiscreteCollisionDetectorInterface
class cbtGjkPairDetector : public cbtDiscreteCollisionDetectorInterface
{
	cbtVector3 m_cachedSeparatingAxis;
	cbtConvexPenetrationDepthSolver* m_penetrationDepthSolver;
	cbtSimplexSolverInterface* m_simplexSolver;
	const cbtConvexShape* m_minkowskiA;
	const cbtConvexShape* m_minkowskiB;
	int m_shapeTypeA;
	int m_shapeTypeB;
	cbtScalar m_marginA;
	cbtScalar m_marginB;

	bool m_ignoreMargin;
	cbtScalar m_cachedSeparatingDistance;

public:
	//some debugging to fix degeneracy problems
	int m_lastUsedMethod;
	int m_curIter;
	int m_degenerateSimplex;
	int m_catchDegeneracies;
	int m_fixContactNormalDirection;

	cbtGjkPairDetector(const cbtConvexShape* objectA, const cbtConvexShape* objectB, cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* penetrationDepthSolver);
	cbtGjkPairDetector(const cbtConvexShape* objectA, const cbtConvexShape* objectB, int shapeTypeA, int shapeTypeB, cbtScalar marginA, cbtScalar marginB, cbtSimplexSolverInterface* simplexSolver, cbtConvexPenetrationDepthSolver* penetrationDepthSolver);
	virtual ~cbtGjkPairDetector(){};

	virtual void getClosestPoints(const ClosestPointInput& input, Result& output, class cbtIDebugDraw* debugDraw, bool swapResults = false);

	void getClosestPointsNonVirtual(const ClosestPointInput& input, Result& output, class cbtIDebugDraw* debugDraw);

	void setMinkowskiA(const cbtConvexShape* minkA)
	{
		m_minkowskiA = minkA;
	}

	void setMinkowskiB(const cbtConvexShape* minkB)
	{
		m_minkowskiB = minkB;
	}
	void setCachedSeperatingAxis(const cbtVector3& seperatingAxis)
	{
		m_cachedSeparatingAxis = seperatingAxis;
	}

	const cbtVector3& getCachedSeparatingAxis() const
	{
		return m_cachedSeparatingAxis;
	}
	cbtScalar getCachedSeparatingDistance() const
	{
		return m_cachedSeparatingDistance;
	}

	void setPenetrationDepthSolver(cbtConvexPenetrationDepthSolver* penetrationDepthSolver)
	{
		m_penetrationDepthSolver = penetrationDepthSolver;
	}

	///don't use setIgnoreMargin, it's for Bullet's internal use
	void setIgnoreMargin(bool ignoreMargin)
	{
		m_ignoreMargin = ignoreMargin;
	}
};

#endif  //BT_GJK_PAIR_DETECTOR_H
