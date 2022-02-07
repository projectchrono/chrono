/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_TRANSFORM_UTIL_H
#define BT_TRANSFORM_UTIL_H

#include "cbtTransform.h"
#define ANGULAR_MOTION_THRESHOLD cbtScalar(0.5) * SIMD_HALF_PI

SIMD_FORCE_INLINE cbtVector3 cbtAabbSupport(const cbtVector3& halfExtents, const cbtVector3& supportDir)
{
	return cbtVector3(supportDir.x() < cbtScalar(0.0) ? -halfExtents.x() : halfExtents.x(),
					 supportDir.y() < cbtScalar(0.0) ? -halfExtents.y() : halfExtents.y(),
					 supportDir.z() < cbtScalar(0.0) ? -halfExtents.z() : halfExtents.z());
}

/// Utils related to temporal transforms
class cbtTransformUtil
{
public:
	static void integrateTransform(const cbtTransform& curTrans, const cbtVector3& linvel, const cbtVector3& angvel, cbtScalar timeStep, cbtTransform& predictedTransform)
	{
		predictedTransform.setOrigin(curTrans.getOrigin() + linvel * timeStep);
		//	#define QUATERNION_DERIVATIVE
#ifdef QUATERNION_DERIVATIVE
		cbtQuaternion predictedOrn = curTrans.getRotation();
		predictedOrn += (angvel * predictedOrn) * (timeStep * cbtScalar(0.5));
		predictedOrn.safeNormalize();
#else
		//Exponential map
		//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

		cbtVector3 axis;
		cbtScalar fAngle2 = angvel.length2();
		cbtScalar fAngle = 0;
		if (fAngle2 > SIMD_EPSILON)
		{
			fAngle = cbtSqrt(fAngle2);
		}

		//limit the angular motion
		if (fAngle * timeStep > ANGULAR_MOTION_THRESHOLD)
		{
			fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
		}

		if (fAngle < cbtScalar(0.001))
		{
			// use Taylor's expansions of sync function
			axis = angvel * (cbtScalar(0.5) * timeStep - (timeStep * timeStep * timeStep) * (cbtScalar(0.020833333333)) * fAngle * fAngle);
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
			axis = angvel * (cbtSin(cbtScalar(0.5) * fAngle * timeStep) / fAngle);
		}
		cbtQuaternion dorn(axis.x(), axis.y(), axis.z(), cbtCos(fAngle * timeStep * cbtScalar(0.5)));
		cbtQuaternion orn0 = curTrans.getRotation();

		cbtQuaternion predictedOrn = dorn * orn0;
		predictedOrn.safeNormalize();
#endif
		if (predictedOrn.length2() > SIMD_EPSILON)
		{
			predictedTransform.setRotation(predictedOrn);
		}
		else
		{
			predictedTransform.setBasis(curTrans.getBasis());
		}
	}

	static void calculateVelocityQuaternion(const cbtVector3& pos0, const cbtVector3& pos1, const cbtQuaternion& orn0, const cbtQuaternion& orn1, cbtScalar timeStep, cbtVector3& linVel, cbtVector3& angVel)
	{
		linVel = (pos1 - pos0) / timeStep;
		cbtVector3 axis;
		cbtScalar angle;
		if (orn0 != orn1)
		{
			calculateDiffAxisAngleQuaternion(orn0, orn1, axis, angle);
			angVel = axis * angle / timeStep;
		}
		else
		{
			angVel.setValue(0, 0, 0);
		}
	}

	static void calculateDiffAxisAngleQuaternion(const cbtQuaternion& orn0, const cbtQuaternion& orn1a, cbtVector3& axis, cbtScalar& angle)
	{
		cbtQuaternion orn1 = orn0.nearest(orn1a);
		cbtQuaternion dorn = orn1 * orn0.inverse();
		angle = dorn.getAngle();
		axis = cbtVector3(dorn.x(), dorn.y(), dorn.z());
		axis[3] = cbtScalar(0.);
		//check for axis length
		cbtScalar len = axis.length2();
		if (len < SIMD_EPSILON * SIMD_EPSILON)
			axis = cbtVector3(cbtScalar(1.), cbtScalar(0.), cbtScalar(0.));
		else
			axis /= cbtSqrt(len);
	}

	static void calculateVelocity(const cbtTransform& transform0, const cbtTransform& transform1, cbtScalar timeStep, cbtVector3& linVel, cbtVector3& angVel)
	{
		linVel = (transform1.getOrigin() - transform0.getOrigin()) / timeStep;
		cbtVector3 axis;
		cbtScalar angle;
		calculateDiffAxisAngle(transform0, transform1, axis, angle);
		angVel = axis * angle / timeStep;
	}

	static void calculateDiffAxisAngle(const cbtTransform& transform0, const cbtTransform& transform1, cbtVector3& axis, cbtScalar& angle)
	{
		cbtMatrix3x3 dmat = transform1.getBasis() * transform0.getBasis().inverse();
		cbtQuaternion dorn;
		dmat.getRotation(dorn);

		///floating point inaccuracy can lead to w component > 1..., which breaks
		dorn.normalize();

		angle = dorn.getAngle();
		axis = cbtVector3(dorn.x(), dorn.y(), dorn.z());
		axis[3] = cbtScalar(0.);
		//check for axis length
		cbtScalar len = axis.length2();
		if (len < SIMD_EPSILON * SIMD_EPSILON)
			axis = cbtVector3(cbtScalar(1.), cbtScalar(0.), cbtScalar(0.));
		else
			axis /= cbtSqrt(len);
	}
};

///The cbtConvexSeparatingDistanceUtil can help speed up convex collision detection
///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
class cbtConvexSeparatingDistanceUtil
{
	cbtQuaternion m_ornA;
	cbtQuaternion m_ornB;
	cbtVector3 m_posA;
	cbtVector3 m_posB;

	cbtVector3 m_separatingNormal;

	cbtScalar m_boundingRadiusA;
	cbtScalar m_boundingRadiusB;
	cbtScalar m_separatingDistance;

public:
	cbtConvexSeparatingDistanceUtil(cbtScalar boundingRadiusA, cbtScalar boundingRadiusB)
		: m_boundingRadiusA(boundingRadiusA),
		  m_boundingRadiusB(boundingRadiusB),
		  m_separatingDistance(0.f)
	{
	}

	cbtScalar getConservativeSeparatingDistance()
	{
		return m_separatingDistance;
	}

	void updateSeparatingDistance(const cbtTransform& transA, const cbtTransform& transB)
	{
		const cbtVector3& toPosA = transA.getOrigin();
		const cbtVector3& toPosB = transB.getOrigin();
		cbtQuaternion toOrnA = transA.getRotation();
		cbtQuaternion toOrnB = transB.getRotation();

		if (m_separatingDistance > 0.f)
		{
			cbtVector3 linVelA, angVelA, linVelB, angVelB;
			cbtTransformUtil::calculateVelocityQuaternion(m_posA, toPosA, m_ornA, toOrnA, cbtScalar(1.), linVelA, angVelA);
			cbtTransformUtil::calculateVelocityQuaternion(m_posB, toPosB, m_ornB, toOrnB, cbtScalar(1.), linVelB, angVelB);
			cbtScalar maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
			cbtVector3 relLinVel = (linVelB - linVelA);
			cbtScalar relLinVelocLength = relLinVel.dot(m_separatingNormal);
			if (relLinVelocLength < 0.f)
			{
				relLinVelocLength = 0.f;
			}

			cbtScalar projectedMotion = maxAngularProjectedVelocity + relLinVelocLength;
			m_separatingDistance -= projectedMotion;
		}

		m_posA = toPosA;
		m_posB = toPosB;
		m_ornA = toOrnA;
		m_ornB = toOrnB;
	}

	void initSeparatingDistance(const cbtVector3& separatingVector, cbtScalar separatingDistance, const cbtTransform& transA, const cbtTransform& transB)
	{
		m_separatingDistance = separatingDistance;

		if (m_separatingDistance > 0.f)
		{
			m_separatingNormal = separatingVector;

			const cbtVector3& toPosA = transA.getOrigin();
			const cbtVector3& toPosB = transB.getOrigin();
			cbtQuaternion toOrnA = transA.getRotation();
			cbtQuaternion toOrnB = transB.getRotation();
			m_posA = toPosA;
			m_posB = toPosB;
			m_ornA = toOrnA;
			m_ornB = toOrnB;
		}
	}
};

#endif  //BT_TRANSFORM_UTIL_H
