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

#ifndef BT_MANIFOLD_RESULT_H
#define BT_MANIFOLD_RESULT_H

class cbtCollisionObject;
struct cbtCollisionObjectWrapper;

#include "BulletCollision/NarrowPhaseCollision/cbtPersistentManifold.h"
class cbtManifoldPoint;

#include "BulletCollision/NarrowPhaseCollision/cbtDiscreteCollisionDetectorInterface.h"

#include "LinearMath/cbtTransform.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObjectWrapper.h"
#include "BulletCollision/CollisionDispatch/cbtCollisionObject.h"

typedef bool (*ContactAddedCallback)(cbtManifoldPoint& cp, const cbtCollisionObjectWrapper* colObj0Wrap, int partId0, int index0, const cbtCollisionObjectWrapper* colObj1Wrap, int partId1, int index1);
extern ContactAddedCallback gContactAddedCallback;

//#define DEBUG_PART_INDEX 1

/// These callbacks are used to customize the algorith that combine restitution, friction, damping, Stiffness
typedef cbtScalar (*CalculateCombinedCallback)(const cbtCollisionObject* body0, const cbtCollisionObject* body1);

extern CalculateCombinedCallback gCalculateCombinedRestitutionCallback;
extern CalculateCombinedCallback gCalculateCombinedFrictionCallback;
extern CalculateCombinedCallback gCalculateCombinedRollingFrictionCallback;
extern CalculateCombinedCallback gCalculateCombinedSpinningFrictionCallback;
extern CalculateCombinedCallback gCalculateCombinedContactDampingCallback;
extern CalculateCombinedCallback gCalculateCombinedContactStiffnessCallback;

///cbtManifoldResult is a helper class to manage  contact results.
class cbtManifoldResult : public cbtDiscreteCollisionDetectorInterface::Result
{
protected:
	cbtPersistentManifold* m_manifoldPtr;

	const cbtCollisionObjectWrapper* m_body0Wrap;
	const cbtCollisionObjectWrapper* m_body1Wrap;
	int m_partId0;
	int m_partId1;
	int m_index0;
	int m_index1;

public:
	cbtManifoldResult()
		:
#ifdef DEBUG_PART_INDEX

		  m_partId0(-1),
		  m_partId1(-1),
		  m_index0(-1),
		  m_index1(-1)
#endif  //DEBUG_PART_INDEX
			  m_closestPointDistanceThreshold(0)
	{
	}

	cbtManifoldResult(const cbtCollisionObjectWrapper* body0Wrap, const cbtCollisionObjectWrapper* body1Wrap);

	virtual ~cbtManifoldResult(){};

	void setPersistentManifold(cbtPersistentManifold* manifoldPtr)
	{
		m_manifoldPtr = manifoldPtr;
	}

	const cbtPersistentManifold* getPersistentManifold() const
	{
		return m_manifoldPtr;
	}
	cbtPersistentManifold* getPersistentManifold()
	{
		return m_manifoldPtr;
	}

	virtual void setShapeIdentifiersA(int partId0, int index0)
	{
		m_partId0 = partId0;
		m_index0 = index0;
	}

	virtual void setShapeIdentifiersB(int partId1, int index1)
	{
		m_partId1 = partId1;
		m_index1 = index1;
	}

	virtual void addContactPoint(const cbtVector3& normalOnBInWorld, const cbtVector3& pointInWorld, cbtScalar depth);

	SIMD_FORCE_INLINE void refreshContactPoints()
	{
		cbtAssert(m_manifoldPtr);
		if (!m_manifoldPtr->getNumContacts())
			return;

		bool isSwapped = m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject();

		if (isSwapped)
		{
			m_manifoldPtr->refreshContactPoints(m_body1Wrap->getCollisionObject()->getWorldTransform(), m_body0Wrap->getCollisionObject()->getWorldTransform());
		}
		else
		{
			m_manifoldPtr->refreshContactPoints(m_body0Wrap->getCollisionObject()->getWorldTransform(), m_body1Wrap->getCollisionObject()->getWorldTransform());
		}
	}

	const cbtCollisionObjectWrapper* getBody0Wrap() const
	{
		return m_body0Wrap;
	}
	const cbtCollisionObjectWrapper* getBody1Wrap() const
	{
		return m_body1Wrap;
	}

	void setBody0Wrap(const cbtCollisionObjectWrapper* obj0Wrap)
	{
		m_body0Wrap = obj0Wrap;
	}

	void setBody1Wrap(const cbtCollisionObjectWrapper* obj1Wrap)
	{
		m_body1Wrap = obj1Wrap;
	}

	const cbtCollisionObject* getBody0Internal() const
	{
		return m_body0Wrap->getCollisionObject();
	}

	const cbtCollisionObject* getBody1Internal() const
	{
		return m_body1Wrap->getCollisionObject();
	}

	cbtScalar m_closestPointDistanceThreshold;

	/// in the future we can let the user override the methods to combine restitution and friction
	static cbtScalar calculateCombinedRestitution(const cbtCollisionObject* body0, const cbtCollisionObject* body1);
	static cbtScalar calculateCombinedFriction(const cbtCollisionObject* body0, const cbtCollisionObject* body1);
	static cbtScalar calculateCombinedRollingFriction(const cbtCollisionObject* body0, const cbtCollisionObject* body1);
	static cbtScalar calculateCombinedSpinningFriction(const cbtCollisionObject* body0, const cbtCollisionObject* body1);
	static cbtScalar calculateCombinedContactDamping(const cbtCollisionObject* body0, const cbtCollisionObject* body1);
	static cbtScalar calculateCombinedContactStiffness(const cbtCollisionObject* body0, const cbtCollisionObject* body1);
};

#endif  //BT_MANIFOLD_RESULT_H
