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

#include "cbtSubSimplexConvexCast.h"
#include "BulletCollision/CollisionShapes/cbtConvexShape.h"

#include "BulletCollision/CollisionShapes/cbtMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/cbtSimplexSolverInterface.h"
#include "cbtPointCollector.h"
#include "LinearMath/cbtTransformUtil.h"

cbtSubsimplexConvexCast::cbtSubsimplexConvexCast(const cbtConvexShape* convexA, const cbtConvexShape* convexB, cbtSimplexSolverInterface* simplexSolver)
	: m_simplexSolver(simplexSolver),
	  m_convexA(convexA),
	  m_convexB(convexB)
{
}


bool cbtSubsimplexConvexCast::calcTimeOfImpact(
	const cbtTransform& fromA,
	const cbtTransform& toA,
	const cbtTransform& fromB,
	const cbtTransform& toB,
	CastResult& result)
{
	m_simplexSolver->reset();

	cbtVector3 linVelA, linVelB;
	linVelA = toA.getOrigin() - fromA.getOrigin();
	linVelB = toB.getOrigin() - fromB.getOrigin();

	cbtScalar lambda = cbtScalar(0.);

	cbtTransform interpolatedTransA = fromA;
	cbtTransform interpolatedTransB = fromB;

	///take relative motion
	cbtVector3 r = (linVelA - linVelB);
	cbtVector3 v;

	cbtVector3 supVertexA = fromA(m_convexA->localGetSupportingVertex(-r * fromA.getBasis()));
	cbtVector3 supVertexB = fromB(m_convexB->localGetSupportingVertex(r * fromB.getBasis()));
	v = supVertexA - supVertexB;
	int maxIter = result.m_subSimplexCastMaxIterations;

	cbtVector3 n;
	n.setValue(cbtScalar(0.), cbtScalar(0.), cbtScalar(0.));

	cbtVector3 c;

	cbtScalar dist2 = v.length2();



	cbtVector3 w, p;
	cbtScalar VdotR;

	while ((dist2 > result.m_subSimplexCastEpsilon) && maxIter--)
	{
		supVertexA = interpolatedTransA(m_convexA->localGetSupportingVertex(-v * interpolatedTransA.getBasis()));
		supVertexB = interpolatedTransB(m_convexB->localGetSupportingVertex(v * interpolatedTransB.getBasis()));
		w = supVertexA - supVertexB;

		cbtScalar VdotW = v.dot(w);

		if (lambda > cbtScalar(1.0))
		{
			return false;
		}

		if (VdotW > cbtScalar(0.))
		{
			VdotR = v.dot(r);

			if (VdotR >= -(SIMD_EPSILON * SIMD_EPSILON))
				return false;
			else
			{
				lambda = lambda - VdotW / VdotR;
				//interpolate to next lambda
				//	x = s + lambda * r;
				interpolatedTransA.getOrigin().setInterpolate3(fromA.getOrigin(), toA.getOrigin(), lambda);
				interpolatedTransB.getOrigin().setInterpolate3(fromB.getOrigin(), toB.getOrigin(), lambda);
				//m_simplexSolver->reset();
				//check next line
				w = supVertexA - supVertexB;

				n = v;
			}
		}
		///Just like regular GJK only add the vertex if it isn't already (close) to current vertex, it would lead to divisions by zero and NaN etc.
		if (!m_simplexSolver->inSimplex(w))
			m_simplexSolver->addVertex(w, supVertexA, supVertexB);

		if (m_simplexSolver->closest(v))
		{
			dist2 = v.length2();

			//todo: check this normal for validity
			//n=v;
			//printf("V=%f , %f, %f\n",v[0],v[1],v[2]);
			//printf("DIST2=%f\n",dist2);
			//printf("numverts = %i\n",m_simplexSolver->numVertices());
		}
		else
		{
			dist2 = cbtScalar(0.);
		}
	}

	//int numiter = MAX_ITERATIONS - maxIter;
	//	printf("number of iterations: %d", numiter);

	//don't report a time of impact when moving 'away' from the hitnormal

	result.m_fraction = lambda;
	if (n.length2() >= (SIMD_EPSILON * SIMD_EPSILON))
		result.m_normal = n.normalized();
	else
		result.m_normal = cbtVector3(cbtScalar(0.0), cbtScalar(0.0), cbtScalar(0.0));

	//don't report time of impact for motion away from the contact normal (or causes minor penetration)
	if (result.m_normal.dot(r) >= -result.m_allowedPenetration)
		return false;

	cbtVector3 hitA, hitB;
	m_simplexSolver->compute_points(hitA, hitB);
	result.m_hitPoint = hitB;
	return true;
}
