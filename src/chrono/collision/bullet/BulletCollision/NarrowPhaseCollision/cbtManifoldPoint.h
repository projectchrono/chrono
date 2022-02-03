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

#ifndef BT_MANIFOLD_CONTACT_POINT_H
#define BT_MANIFOLD_CONTACT_POINT_H

#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtTransformUtil.h"

#ifdef PFX_USE_FREE_VECTORMATH
#include "physics_effects/base_level/solver/pfx_constraint_row.h"
typedef sce::PhysicsEffects::PfxConstraintRow cbtConstraintRow;
#else
// Don't change following order of parameters
ATTRIBUTE_ALIGNED16(struct)
cbtConstraintRow
{
	cbtScalar m_normal[3];
	cbtScalar m_rhs;
	cbtScalar m_jacDiagInv;
	cbtScalar m_lowerLimit;
	cbtScalar m_upperLimit;
	cbtScalar m_accumImpulse;
};
typedef cbtConstraintRow PfxConstraintRow;
#endif  //PFX_USE_FREE_VECTORMATH

enum cbtContactPointFlags
{
	BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED = 1,
	BT_CONTACT_FLAG_HAS_CONTACT_CFM = 2,
	BT_CONTACT_FLAG_HAS_CONTACT_ERP = 4,
	BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING = 8,
	BT_CONTACT_FLAG_FRICTION_ANCHOR = 16,
};

/// ManifoldContactPoint collects and maintains persistent contactpoints.
/// used to improve stability and performance of rigidbody dynamics response.
class cbtManifoldPoint
{
public:
	cbtManifoldPoint()
		: m_userPersistentData(0),
		  m_contactPointFlags(0),
		  m_appliedImpulse(0.f),
		  m_appliedImpulseLateral1(0.f),
		  m_appliedImpulseLateral2(0.f),
		  m_contactMotion1(0.f),
		  m_contactMotion2(0.f),
		  m_contactCFM(0.f),
		  m_contactERP(0.f),
		  m_frictionCFM(0.f),
		  m_lifeTime(0)
	{
		// ***CHRONO***
		reactions_cache[0]=reactions_cache[1]=reactions_cache[2]=reactions_cache[3]=reactions_cache[4]=reactions_cache[5]=0;
	}

	cbtManifoldPoint(const cbtVector3& pointA, const cbtVector3& pointB,
					const cbtVector3& normal,
					cbtScalar distance) : m_localPointA(pointA),
										 m_localPointB(pointB),
										 m_normalWorldOnB(normal),
										 m_distance1(distance),
										 m_combinedFriction(cbtScalar(0.)),
										 m_combinedRollingFriction(cbtScalar(0.)),
										 m_combinedSpinningFriction(cbtScalar(0.)),
										 m_combinedRestitution(cbtScalar(0.)),
										 m_userPersistentData(0),
										 m_contactPointFlags(0),
										 m_appliedImpulse(0.f),
										 m_appliedImpulseLateral1(0.f),
										 m_appliedImpulseLateral2(0.f),
										 m_contactMotion1(0.f),
										 m_contactMotion2(0.f),
										 m_contactCFM(0.f),
										 m_contactERP(0.f),
										 m_frictionCFM(0.f),
										 m_lifeTime(0)
	{
		// ***CHRONO***
		reactions_cache[0]=reactions_cache[1]=reactions_cache[2]=reactions_cache[3]=reactions_cache[4]=reactions_cache[5]=0;
	}

	float reactions_cache[6]; // ***CHRONO***  cache here the three multipliers N,U,V for warm starting the NCP solver.

	cbtVector3 m_localPointA;
	cbtVector3 m_localPointB;
	cbtVector3 m_positionWorldOnB;
	///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
	cbtVector3 m_positionWorldOnA;
	cbtVector3 m_normalWorldOnB;

	cbtScalar m_distance1;
	cbtScalar m_combinedFriction;
	cbtScalar m_combinedRollingFriction;   //torsional friction orthogonal to contact normal, useful to make spheres stop rolling forever
	cbtScalar m_combinedSpinningFriction;  //torsional friction around contact normal, useful for grasping objects
	cbtScalar m_combinedRestitution;

	//BP mod, store contact triangles.
	int m_partId0;
	int m_partId1;
	int m_index0;
	int m_index1;

	mutable void* m_userPersistentData;
	//bool			m_lateralFrictionInitialized;
	int m_contactPointFlags;

	cbtScalar m_appliedImpulse;
	cbtScalar m_appliedImpulseLateral1;
	cbtScalar m_appliedImpulseLateral2;
	cbtScalar m_contactMotion1;
	cbtScalar m_contactMotion2;

	union {
		cbtScalar m_contactCFM;
		cbtScalar m_combinedContactStiffness1;
	};

	union {
		cbtScalar m_contactERP;
		cbtScalar m_combinedContactDamping1;
	};

	cbtScalar m_frictionCFM;

	int m_lifeTime;  //lifetime of the contactpoint in frames

	cbtVector3 m_lateralFrictionDir1;
	cbtVector3 m_lateralFrictionDir2;

	cbtScalar getDistance() const
	{
		return m_distance1;
	}
	int getLifeTime() const
	{
		return m_lifeTime;
	}

	const cbtVector3& getPositionWorldOnA() const
	{
		return m_positionWorldOnA;
		//				return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
	}

	const cbtVector3& getPositionWorldOnB() const
	{
		return m_positionWorldOnB;
	}

	void setDistance(cbtScalar dist)
	{
		m_distance1 = dist;
	}

	///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
	cbtScalar getAppliedImpulse() const
	{
		return m_appliedImpulse;
	}
};

#endif  //BT_MANIFOLD_CONTACT_POINT_H
