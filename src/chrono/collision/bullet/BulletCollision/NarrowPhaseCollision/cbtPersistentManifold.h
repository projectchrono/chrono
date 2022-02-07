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

#ifndef BT_PERSISTENT_MANIFOLD_H
#define BT_PERSISTENT_MANIFOLD_H

#include "LinearMath/cbtVector3.h"
#include "LinearMath/cbtTransform.h"
#include "cbtManifoldPoint.h"
class cbtCollisionObject;
#include "LinearMath/cbtAlignedAllocator.h"

struct cbtCollisionResult;
struct cbtCollisionObjectDoubleData;
struct cbtCollisionObjectFloatData;

///maximum contact breaking and merging threshold
extern cbtScalar gContactBreakingThreshold;

#ifndef SWIG
class cbtPersistentManifold;

typedef bool (*ContactDestroyedCallback)(void* userPersistentData);
typedef bool (*ContactProcessedCallback)(cbtManifoldPoint& cp, void* body0, void* body1);
typedef void (*ContactStartedCallback)(cbtPersistentManifold* const& manifold);
typedef void (*ContactEndedCallback)(cbtPersistentManifold* const& manifold);
extern ContactDestroyedCallback gContactDestroyedCallback;
extern ContactProcessedCallback gContactProcessedCallback;
extern ContactStartedCallback gContactStartedCallback;
extern ContactEndedCallback gContactEndedCallback;
#endif  //SWIG

//the enum starts at 1024 to avoid type conflicts with cbtTypedConstraint
enum cbtContactManifoldTypes
{
	MIN_CONTACT_MANIFOLD_TYPE = 1024,
	BT_PERSISTENT_MANIFOLD_TYPE
};

#define MANIFOLD_CACHE_SIZE 4

///cbtPersistentManifold is a contact point cache, it stays persistent as long as objects are overlapping in the broadphase.
///Those contact points are created by the collision narrow phase.
///The cache can be empty, or hold 1,2,3 or 4 points. Some collision algorithms (GJK) might only add one point at a time.
///updates/refreshes old contact points, and throw them away if necessary (distance becomes too large)
///reduces the cache to 4 points, when more then 4 points are added, using following rules:
///the contact point with deepest penetration is always kept, and it tries to maximuze the area covered by the points
///note that some pairs of objects might have more then one contact manifold.

//ATTRIBUTE_ALIGNED128( class) cbtPersistentManifold : public cbtTypedObject
ATTRIBUTE_ALIGNED16(class)
cbtPersistentManifold : public cbtTypedObject
{
	cbtManifoldPoint m_pointCache[MANIFOLD_CACHE_SIZE];

	/// this two body pointers can point to the physics rigidbody class.
	const cbtCollisionObject* m_body0;
	const cbtCollisionObject* m_body1;

	int m_cachedPoints;

	cbtScalar m_contactBreakingThreshold;
	cbtScalar m_contactProcessingThreshold;

	/// sort cached points so most isolated points come first
	int sortCachedPoints(const cbtManifoldPoint& pt);

	int findContactPoint(const cbtManifoldPoint* unUsed, int numUnused, const cbtManifoldPoint& pt);

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	int m_companionIdA;
	int m_companionIdB;

	int m_index1a;

	cbtPersistentManifold();

	cbtPersistentManifold(const cbtCollisionObject* body0, const cbtCollisionObject* body1, int, cbtScalar contactBreakingThreshold, cbtScalar contactProcessingThreshold)
		: cbtTypedObject(BT_PERSISTENT_MANIFOLD_TYPE),
		  m_body0(body0),
		  m_body1(body1),
		  m_cachedPoints(0),
		  m_contactBreakingThreshold(contactBreakingThreshold),
		  m_contactProcessingThreshold(contactProcessingThreshold),
		  m_companionIdA(0),
		  m_companionIdB(0),
		  m_index1a(0)
	{
	}

	SIMD_FORCE_INLINE const cbtCollisionObject* getBody0() const { return m_body0; }
	SIMD_FORCE_INLINE const cbtCollisionObject* getBody1() const { return m_body1; }

	void setBodies(const cbtCollisionObject* body0, const cbtCollisionObject* body1)
	{
		m_body0 = body0;
		m_body1 = body1;
	}

	void clearUserCache(cbtManifoldPoint & pt);

#ifdef DEBUG_PERSISTENCY
	void DebugPersistency();
#endif  //

	SIMD_FORCE_INLINE int getNumContacts() const
	{
		return m_cachedPoints;
	}
	/// the setNumContacts API is usually not used, except when you gather/fill all contacts manually
	void setNumContacts(int cachedPoints)
	{
		m_cachedPoints = cachedPoints;
	}

	SIMD_FORCE_INLINE const cbtManifoldPoint& getContactPoint(int index) const
	{
		cbtAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	SIMD_FORCE_INLINE cbtManifoldPoint& getContactPoint(int index)
	{
		cbtAssert(index < m_cachedPoints);
		return m_pointCache[index];
	}

	///@todo: get this margin from the current physics / collision environment
	cbtScalar getContactBreakingThreshold() const;

	cbtScalar getContactProcessingThreshold() const
	{
		return m_contactProcessingThreshold;
	}

	void setContactBreakingThreshold(cbtScalar contactBreakingThreshold)
	{
		m_contactBreakingThreshold = contactBreakingThreshold;
	}

	void setContactProcessingThreshold(cbtScalar contactProcessingThreshold)
	{
		m_contactProcessingThreshold = contactProcessingThreshold;
	}

	int getCacheEntry(const cbtManifoldPoint& newPoint) const;

	int addManifoldPoint(const cbtManifoldPoint& newPoint, bool isPredictive = false);

	void removeContactPoint(int index)
	{
		clearUserCache(m_pointCache[index]);

		int lastUsedIndex = getNumContacts() - 1;
		//		m_pointCache[index] = m_pointCache[lastUsedIndex];
		if (index != lastUsedIndex)
		{
			m_pointCache[index] = m_pointCache[lastUsedIndex];
			//get rid of duplicated userPersistentData pointer
			m_pointCache[lastUsedIndex].m_userPersistentData = 0;
			m_pointCache[lastUsedIndex].m_appliedImpulse = 0.f;
			m_pointCache[lastUsedIndex].m_contactPointFlags = 0;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral1 = 0.f;
			m_pointCache[lastUsedIndex].m_appliedImpulseLateral2 = 0.f;
			m_pointCache[lastUsedIndex].m_lifeTime = 0;

			m_pointCache[lastUsedIndex].reactions_cache[0] = 0;  // ***CHRONO***
			m_pointCache[lastUsedIndex].reactions_cache[1] = 0;
			m_pointCache[lastUsedIndex].reactions_cache[2] = 0;
			m_pointCache[lastUsedIndex].reactions_cache[3] = 0;
			m_pointCache[lastUsedIndex].reactions_cache[4] = 0;
			m_pointCache[lastUsedIndex].reactions_cache[5] = 0;
		}

		cbtAssert(m_pointCache[lastUsedIndex].m_userPersistentData == 0);
		m_cachedPoints--;

		if (gContactEndedCallback && m_cachedPoints == 0)
		{
			gContactEndedCallback(this);
		}
	}
	void replaceContactPoint(const cbtManifoldPoint& newPoint, int insertIndex)
	{
		cbtAssert(validContactDistance(newPoint));

#define MAINTAIN_PERSISTENCY 1
#ifdef MAINTAIN_PERSISTENCY
		int lifeTime = m_pointCache[insertIndex].getLifeTime();
		cbtScalar appliedImpulse = m_pointCache[insertIndex].m_appliedImpulse;
		cbtScalar appliedLateralImpulse1 = m_pointCache[insertIndex].m_appliedImpulseLateral1;
		cbtScalar appliedLateralImpulse2 = m_pointCache[insertIndex].m_appliedImpulseLateral2;

		bool replacePoint = true;
		///we keep existing contact points for friction anchors
		///if the friction force is within the Coulomb friction cone
		if (newPoint.m_contactPointFlags & BT_CONTACT_FLAG_FRICTION_ANCHOR)
		{
			//   printf("appliedImpulse=%f\n", appliedImpulse);
			//   printf("appliedLateralImpulse1=%f\n", appliedLateralImpulse1);
			//   printf("appliedLateralImpulse2=%f\n", appliedLateralImpulse2);
			//   printf("mu = %f\n", m_pointCache[insertIndex].m_combinedFriction);
			cbtScalar mu = m_pointCache[insertIndex].m_combinedFriction;
			cbtScalar eps = 0;  //we could allow to enlarge or shrink the tolerance to check against the friction cone a bit, say 1e-7
			cbtScalar a = appliedLateralImpulse1 * appliedLateralImpulse1 + appliedLateralImpulse2 * appliedLateralImpulse2;
			cbtScalar b = eps + mu * appliedImpulse;
			b = b * b;
			replacePoint = (a) > (b);
		}

		if (replacePoint)
		{
			cbtAssert(lifeTime >= 0);
			void* cache = m_pointCache[insertIndex].m_userPersistentData;

			float mx = m_pointCache[insertIndex].reactions_cache[0]; // ***CHRONO***
			float my = m_pointCache[insertIndex].reactions_cache[1];
			float mz = m_pointCache[insertIndex].reactions_cache[2];
			float mf = m_pointCache[insertIndex].reactions_cache[3];
			float mg = m_pointCache[insertIndex].reactions_cache[4];
			float mh = m_pointCache[insertIndex].reactions_cache[5];

			m_pointCache[insertIndex] = newPoint;
			m_pointCache[insertIndex].m_userPersistentData = cache;
			m_pointCache[insertIndex].m_appliedImpulse = appliedImpulse;
			m_pointCache[insertIndex].m_appliedImpulseLateral1 = appliedLateralImpulse1;
			m_pointCache[insertIndex].m_appliedImpulseLateral2 = appliedLateralImpulse2;

			m_pointCache[insertIndex].reactions_cache[0] = mx;  // ***CHRONO***
			m_pointCache[insertIndex].reactions_cache[1] = my;
			m_pointCache[insertIndex].reactions_cache[2] = mz;
			m_pointCache[insertIndex].reactions_cache[3] = mf;
			m_pointCache[insertIndex].reactions_cache[4] = mg;
			m_pointCache[insertIndex].reactions_cache[5] = mh;
		}

		m_pointCache[insertIndex].m_lifeTime = lifeTime;
#else
		clearUserCache(m_pointCache[insertIndex]);
		m_pointCache[insertIndex] = newPoint;

#endif
	}

	bool validContactDistance(const cbtManifoldPoint& pt) const
	{
		return pt.m_distance1 <= getContactBreakingThreshold();
	}
	/// calculated new worldspace coordinates and depth, and reject points that exceed the collision margin
	void refreshContactPoints(const cbtTransform& trA, const cbtTransform& trB);

	SIMD_FORCE_INLINE void clearManifold()
	{
		int i;
		for (i = 0; i < m_cachedPoints; i++)
		{
			clearUserCache(m_pointCache[i]);
		}

		if (gContactEndedCallback && m_cachedPoints)
		{
			gContactEndedCallback(this);
		}
		m_cachedPoints = 0;
	}

	int calculateSerializeBufferSize() const;
	const char* serialize(const class cbtPersistentManifold* manifold, void* dataBuffer, class cbtSerializer* serializer) const;
	void deSerialize(const struct cbtPersistentManifoldDoubleData* manifoldDataPtr);
	void deSerialize(const struct cbtPersistentManifoldFloatData* manifoldDataPtr);
};

// clang-format off

struct cbtPersistentManifoldDoubleData
{
	cbtVector3DoubleData m_pointCacheLocalPointA[4];
	cbtVector3DoubleData m_pointCacheLocalPointB[4];
	cbtVector3DoubleData m_pointCachePositionWorldOnA[4];
	cbtVector3DoubleData m_pointCachePositionWorldOnB[4];
	cbtVector3DoubleData m_pointCacheNormalWorldOnB[4];
	cbtVector3DoubleData	m_pointCacheLateralFrictionDir1[4];
	cbtVector3DoubleData	m_pointCacheLateralFrictionDir2[4];
	double m_pointCacheDistance[4];
	double m_pointCacheAppliedImpulse[4];
	double m_pointCacheCombinedFriction[4];
	double m_pointCacheCombinedRollingFriction[4];
	double m_pointCacheCombinedSpinningFriction[4];
	double m_pointCacheCombinedRestitution[4];
	int	m_pointCachePartId0[4];
	int	m_pointCachePartId1[4];
	int	m_pointCacheIndex0[4];
	int	m_pointCacheIndex1[4];
	int m_pointCacheContactPointFlags[4];
	double m_pointCacheAppliedImpulseLateral1[4];
	double m_pointCacheAppliedImpulseLateral2[4];
	double m_pointCacheContactMotion1[4];
	double m_pointCacheContactMotion2[4];
	double m_pointCacheContactCFM[4];
	double m_pointCacheCombinedContactStiffness1[4];
	double m_pointCacheContactERP[4];
	double m_pointCacheCombinedContactDamping1[4];
	double m_pointCacheFrictionCFM[4];
	int m_pointCacheLifeTime[4];

	int m_numCachedPoints;
	int m_companionIdA;
	int m_companionIdB;
	int m_index1a;

	int m_objectType;
	double	m_contactBreakingThreshold;
	double	m_contactProcessingThreshold;
	int m_padding;

	cbtCollisionObjectDoubleData *m_body0;
	cbtCollisionObjectDoubleData *m_body1;
};


struct cbtPersistentManifoldFloatData
{
	cbtVector3FloatData m_pointCacheLocalPointA[4];
	cbtVector3FloatData m_pointCacheLocalPointB[4];
	cbtVector3FloatData m_pointCachePositionWorldOnA[4];
	cbtVector3FloatData m_pointCachePositionWorldOnB[4];
	cbtVector3FloatData m_pointCacheNormalWorldOnB[4];
	cbtVector3FloatData	m_pointCacheLateralFrictionDir1[4];
	cbtVector3FloatData	m_pointCacheLateralFrictionDir2[4];
	float m_pointCacheDistance[4];
	float m_pointCacheAppliedImpulse[4];
	float m_pointCacheCombinedFriction[4];
	float m_pointCacheCombinedRollingFriction[4];
	float m_pointCacheCombinedSpinningFriction[4];
	float m_pointCacheCombinedRestitution[4];
	int	m_pointCachePartId0[4];
	int	m_pointCachePartId1[4];
	int	m_pointCacheIndex0[4];
	int	m_pointCacheIndex1[4];
	int m_pointCacheContactPointFlags[4];
	float m_pointCacheAppliedImpulseLateral1[4];
	float m_pointCacheAppliedImpulseLateral2[4];
	float m_pointCacheContactMotion1[4];
	float m_pointCacheContactMotion2[4];
	float m_pointCacheContactCFM[4];
	float m_pointCacheCombinedContactStiffness1[4];
	float m_pointCacheContactERP[4];
	float m_pointCacheCombinedContactDamping1[4];
	float m_pointCacheFrictionCFM[4];
	int m_pointCacheLifeTime[4];

	int m_numCachedPoints;
	int m_companionIdA;
	int m_companionIdB;
	int m_index1a;

	int m_objectType;
	float	m_contactBreakingThreshold;
	float	m_contactProcessingThreshold;
	int m_padding;

	cbtCollisionObjectFloatData *m_body0;
	cbtCollisionObjectFloatData *m_body1;
};

// clang-format on

#ifdef BT_USE_DOUBLE_PRECISION
#define cbtPersistentManifoldData cbtPersistentManifoldDoubleData
#define cbtPersistentManifoldDataName "cbtPersistentManifoldDoubleData"
#else
#define cbtPersistentManifoldData cbtPersistentManifoldFloatData
#define cbtPersistentManifoldDataName "cbtPersistentManifoldFloatData"
#endif  //BT_USE_DOUBLE_PRECISION

#endif  //BT_PERSISTENT_MANIFOLD_H
