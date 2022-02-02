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

#include "cbtPersistentManifold.h"
#include "LinearMath/cbtTransform.h"
#include "LinearMath/cbtSerializer.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define cbtCollisionObjectData cbtCollisionObjectDoubleData
#else
#define cbtCollisionObjectData cbtCollisionObjectFloatData
#endif

/* ***CHRONO***, 0.004, original 0.02, warning: do not set too low or you get troubles with aabb */
cbtScalar gContactBreakingThreshold = cbtScalar(0.004);
ContactDestroyedCallback gContactDestroyedCallback = 0;
ContactProcessedCallback gContactProcessedCallback = 0;
ContactStartedCallback gContactStartedCallback = 0;
ContactEndedCallback gContactEndedCallback = 0;
///gContactCalcArea3Points will approximate the convex hull area using 3 points
///when setting it to false, it will use 4 points to compute the area: it is more accurate but slower
bool gContactCalcArea3Points = true;

cbtPersistentManifold::cbtPersistentManifold()
	: cbtTypedObject(BT_PERSISTENT_MANIFOLD_TYPE),
	  m_body0(0),
	  m_body1(0),
	  m_cachedPoints(0),
	  m_companionIdA(0),
	  m_companionIdB(0),
	  m_index1a(0)
{
}

#ifdef DEBUG_PERSISTENCY
#include <stdio.h>
void cbtPersistentManifold::DebugPersistency()
{
	int i;
	printf("DebugPersistency : numPoints %d\n", m_cachedPoints);
	for (i = 0; i < m_cachedPoints; i++)
	{
		printf("m_pointCache[%d].m_userPersistentData = %x\n", i, m_pointCache[i].m_userPersistentData);
	}
}
#endif  //DEBUG_PERSISTENCY

void cbtPersistentManifold::clearUserCache(cbtManifoldPoint& pt) {
    /* ***CHRONO*** */
    pt.reactions_cache[0] = pt.reactions_cache[1] = pt.reactions_cache[2] = 0;
    pt.reactions_cache[3] = pt.reactions_cache[4] = pt.reactions_cache[5] = 0;

	void* oldPtr = pt.m_userPersistentData;
	if (oldPtr)
	{
#ifdef DEBUG_PERSISTENCY
		int i;
		int occurance = 0;
		for (i = 0; i < m_cachedPoints; i++)
		{
			if (m_pointCache[i].m_userPersistentData == oldPtr)
			{
				occurance++;
				if (occurance > 1)
					printf("error in clearUserCache\n");
			}
		}
		cbtAssert(occurance <= 0);
#endif  //DEBUG_PERSISTENCY

		if (pt.m_userPersistentData && gContactDestroyedCallback)
		{
			(*gContactDestroyedCallback)(pt.m_userPersistentData);
			pt.m_userPersistentData = 0;
		}

#ifdef DEBUG_PERSISTENCY
		DebugPersistency();
#endif
	}
}

static inline cbtScalar calcArea4Points(const cbtVector3& p0, const cbtVector3& p1, const cbtVector3& p2, const cbtVector3& p3)
{
	// It calculates possible 3 area constructed from random 4 points and returns the biggest one.

	cbtVector3 a[3], b[3];
	a[0] = p0 - p1;
	a[1] = p0 - p2;
	a[2] = p0 - p3;
	b[0] = p2 - p3;
	b[1] = p1 - p3;
	b[2] = p1 - p2;

	//todo: Following 3 cross production can be easily optimized by SIMD.
	cbtVector3 tmp0 = a[0].cross(b[0]);
	cbtVector3 tmp1 = a[1].cross(b[1]);
	cbtVector3 tmp2 = a[2].cross(b[2]);

	return cbtMax(cbtMax(tmp0.length2(), tmp1.length2()), tmp2.length2());
}

int cbtPersistentManifold::sortCachedPoints(const cbtManifoldPoint& pt)
{
	//calculate 4 possible cases areas, and take biggest area
	//also need to keep 'deepest'

	int maxPenetrationIndex = -1;
#define KEEP_DEEPEST_POINT 1
#ifdef KEEP_DEEPEST_POINT
	cbtScalar maxPenetration = pt.getDistance();
	for (int i = 0; i < 4; i++)
	{
		if (m_pointCache[i].getDistance() < maxPenetration)
		{
			maxPenetrationIndex = i;
			maxPenetration = m_pointCache[i].getDistance();
		}
	}
#endif  //KEEP_DEEPEST_POINT

	cbtScalar res0(cbtScalar(0.)), res1(cbtScalar(0.)), res2(cbtScalar(0.)), res3(cbtScalar(0.));

	if (gContactCalcArea3Points)
	{
		if (maxPenetrationIndex != 0)
		{
			cbtVector3 a0 = pt.m_localPointA - m_pointCache[1].m_localPointA;
			cbtVector3 b0 = m_pointCache[3].m_localPointA - m_pointCache[2].m_localPointA;
			cbtVector3 cross = a0.cross(b0);
			res0 = cross.length2();
		}
		if (maxPenetrationIndex != 1)
		{
			cbtVector3 a1 = pt.m_localPointA - m_pointCache[0].m_localPointA;
			cbtVector3 b1 = m_pointCache[3].m_localPointA - m_pointCache[2].m_localPointA;
			cbtVector3 cross = a1.cross(b1);
			res1 = cross.length2();
		}

		if (maxPenetrationIndex != 2)
		{
			cbtVector3 a2 = pt.m_localPointA - m_pointCache[0].m_localPointA;
			cbtVector3 b2 = m_pointCache[3].m_localPointA - m_pointCache[1].m_localPointA;
			cbtVector3 cross = a2.cross(b2);
			res2 = cross.length2();
		}

		if (maxPenetrationIndex != 3)
		{
			cbtVector3 a3 = pt.m_localPointA - m_pointCache[0].m_localPointA;
			cbtVector3 b3 = m_pointCache[2].m_localPointA - m_pointCache[1].m_localPointA;
			cbtVector3 cross = a3.cross(b3);
			res3 = cross.length2();
		}
	}
	else
	{
		if (maxPenetrationIndex != 0)
		{
			res0 = calcArea4Points(pt.m_localPointA, m_pointCache[1].m_localPointA, m_pointCache[2].m_localPointA, m_pointCache[3].m_localPointA);
		}

		if (maxPenetrationIndex != 1)
		{
			res1 = calcArea4Points(pt.m_localPointA, m_pointCache[0].m_localPointA, m_pointCache[2].m_localPointA, m_pointCache[3].m_localPointA);
		}

		if (maxPenetrationIndex != 2)
		{
			res2 = calcArea4Points(pt.m_localPointA, m_pointCache[0].m_localPointA, m_pointCache[1].m_localPointA, m_pointCache[3].m_localPointA);
		}

		if (maxPenetrationIndex != 3)
		{
			res3 = calcArea4Points(pt.m_localPointA, m_pointCache[0].m_localPointA, m_pointCache[1].m_localPointA, m_pointCache[2].m_localPointA);
		}
	}
	cbtVector4 maxvec(res0, res1, res2, res3);
	int biggestarea = maxvec.closestAxis4();
	return biggestarea;
}

int cbtPersistentManifold::getCacheEntry(const cbtManifoldPoint& newPoint) const
{
	cbtScalar shortestDist = getContactBreakingThreshold() * getContactBreakingThreshold();
	int size = getNumContacts();
	int nearestPoint = -1;
	for (int i = 0; i < size; i++)
	{
		const cbtManifoldPoint& mp = m_pointCache[i];

		cbtVector3 diffA = mp.m_localPointA - newPoint.m_localPointA;
		const cbtScalar distToManiPoint = diffA.dot(diffA);
		if (distToManiPoint < shortestDist)
		{
			shortestDist = distToManiPoint;
			nearestPoint = i;
		}
	}
	return nearestPoint;
}

int cbtPersistentManifold::addManifoldPoint(const cbtManifoldPoint& newPoint, bool isPredictive)
{
	if (!isPredictive)
	{
		cbtAssert(validContactDistance(newPoint));
	}

	int insertIndex = getNumContacts();
	if (insertIndex == MANIFOLD_CACHE_SIZE)
	{
#if MANIFOLD_CACHE_SIZE >= 4
		//sort cache so best points come first, based on area
		insertIndex = sortCachedPoints(newPoint);
#else
		insertIndex = 0;
#endif
		clearUserCache(m_pointCache[insertIndex]);
	}
	else
	{
		m_cachedPoints++;
	}
	if (insertIndex < 0)
		insertIndex = 0;

	cbtAssert(m_pointCache[insertIndex].m_userPersistentData == 0);
	m_pointCache[insertIndex] = newPoint;
	return insertIndex;
}

cbtScalar cbtPersistentManifold::getContactBreakingThreshold() const
{
	return m_contactBreakingThreshold;
}

void cbtPersistentManifold::refreshContactPoints(const cbtTransform& trA, const cbtTransform& trB)
{
	int i;
#ifdef DEBUG_PERSISTENCY
	printf("refreshContactPoints posA = (%f,%f,%f) posB = (%f,%f,%f)\n",
		   trA.getOrigin().getX(),
		   trA.getOrigin().getY(),
		   trA.getOrigin().getZ(),
		   trB.getOrigin().getX(),
		   trB.getOrigin().getY(),
		   trB.getOrigin().getZ());
#endif  //DEBUG_PERSISTENCY
	/// first refresh worldspace positions and distance
	for (i = getNumContacts() - 1; i >= 0; i--)
	{
		cbtManifoldPoint& manifoldPoint = m_pointCache[i];
		manifoldPoint.m_positionWorldOnA = trA(manifoldPoint.m_localPointA);
		manifoldPoint.m_positionWorldOnB = trB(manifoldPoint.m_localPointB);
		manifoldPoint.m_distance1 = (manifoldPoint.m_positionWorldOnA - manifoldPoint.m_positionWorldOnB).dot(manifoldPoint.m_normalWorldOnB);
		manifoldPoint.m_lifeTime++;
	}

	/// then
	cbtScalar distance2d;
	cbtVector3 projectedDifference, projectedPoint;
	for (i = getNumContacts() - 1; i >= 0; i--)
	{
		cbtManifoldPoint& manifoldPoint = m_pointCache[i];
		//contact becomes invalid when signed distance exceeds margin (projected on contactnormal direction)
		if (!validContactDistance(manifoldPoint))
		{
			removeContactPoint(i);
		}
		else
		{
			//todo: friction anchor may require the contact to be around a bit longer
			//contact also becomes invalid when relative movement orthogonal to normal exceeds margin
			projectedPoint = manifoldPoint.m_positionWorldOnA - manifoldPoint.m_normalWorldOnB * manifoldPoint.m_distance1;
			projectedDifference = manifoldPoint.m_positionWorldOnB - projectedPoint;
			distance2d = projectedDifference.dot(projectedDifference);
			if (distance2d > getContactBreakingThreshold() * getContactBreakingThreshold())
			{
				removeContactPoint(i);
			}
			else
			{
				//contact point processed callback
				if (gContactProcessedCallback)
					(*gContactProcessedCallback)(manifoldPoint, (void*)m_body0, (void*)m_body1);
			}
		}
	}
#ifdef DEBUG_PERSISTENCY
	DebugPersistency();
#endif  //
}

int cbtPersistentManifold::calculateSerializeBufferSize() const
{
	return sizeof(cbtPersistentManifoldData);
}

const char* cbtPersistentManifold::serialize(const class cbtPersistentManifold* manifold, void* dataBuffer, class cbtSerializer* serializer) const
{
	cbtPersistentManifoldData* dataOut = (cbtPersistentManifoldData*)dataBuffer;
	memset(dataOut, 0, sizeof(cbtPersistentManifoldData));

	dataOut->m_body0 = (cbtCollisionObjectData*)serializer->getUniquePointer((void*)manifold->getBody0());
	dataOut->m_body1 = (cbtCollisionObjectData*)serializer->getUniquePointer((void*)manifold->getBody1());
	dataOut->m_contactBreakingThreshold = manifold->getContactBreakingThreshold();
	dataOut->m_contactProcessingThreshold = manifold->getContactProcessingThreshold();
	dataOut->m_numCachedPoints = manifold->getNumContacts();
	dataOut->m_companionIdA = manifold->m_companionIdA;
	dataOut->m_companionIdB = manifold->m_companionIdB;
	dataOut->m_index1a = manifold->m_index1a;
	dataOut->m_objectType = manifold->m_objectType;

	for (int i = 0; i < this->getNumContacts(); i++)
	{
		const cbtManifoldPoint& pt = manifold->getContactPoint(i);
		dataOut->m_pointCacheAppliedImpulse[i] = pt.m_appliedImpulse;
		dataOut->m_pointCacheAppliedImpulseLateral1[i] = pt.m_appliedImpulseLateral1;
		dataOut->m_pointCacheAppliedImpulseLateral2[i] = pt.m_appliedImpulseLateral2;
		pt.m_localPointA.serialize(dataOut->m_pointCacheLocalPointA[i]);
		pt.m_localPointB.serialize(dataOut->m_pointCacheLocalPointB[i]);
		pt.m_normalWorldOnB.serialize(dataOut->m_pointCacheNormalWorldOnB[i]);
		dataOut->m_pointCacheDistance[i] = pt.m_distance1;
		dataOut->m_pointCacheCombinedContactDamping1[i] = pt.m_combinedContactDamping1;
		dataOut->m_pointCacheCombinedContactStiffness1[i] = pt.m_combinedContactStiffness1;
		dataOut->m_pointCacheLifeTime[i] = pt.m_lifeTime;
		dataOut->m_pointCacheFrictionCFM[i] = pt.m_frictionCFM;
		dataOut->m_pointCacheContactERP[i] = pt.m_contactERP;
		dataOut->m_pointCacheContactCFM[i] = pt.m_contactCFM;
		dataOut->m_pointCacheContactPointFlags[i] = pt.m_contactPointFlags;
		dataOut->m_pointCacheIndex0[i] = pt.m_index0;
		dataOut->m_pointCacheIndex1[i] = pt.m_index1;
		dataOut->m_pointCachePartId0[i] = pt.m_partId0;
		dataOut->m_pointCachePartId1[i] = pt.m_partId1;
		pt.m_positionWorldOnA.serialize(dataOut->m_pointCachePositionWorldOnA[i]);
		pt.m_positionWorldOnB.serialize(dataOut->m_pointCachePositionWorldOnB[i]);
		dataOut->m_pointCacheCombinedFriction[i] = pt.m_combinedFriction;
		pt.m_lateralFrictionDir1.serialize(dataOut->m_pointCacheLateralFrictionDir1[i]);
		pt.m_lateralFrictionDir2.serialize(dataOut->m_pointCacheLateralFrictionDir2[i]);
		dataOut->m_pointCacheCombinedRollingFriction[i] = pt.m_combinedRollingFriction;
		dataOut->m_pointCacheCombinedSpinningFriction[i] = pt.m_combinedSpinningFriction;
		dataOut->m_pointCacheCombinedRestitution[i] = pt.m_combinedRestitution;
		dataOut->m_pointCacheContactMotion1[i] = pt.m_contactMotion1;
		dataOut->m_pointCacheContactMotion2[i] = pt.m_contactMotion2;
	}
	return cbtPersistentManifoldDataName;
}

void cbtPersistentManifold::deSerialize(const struct cbtPersistentManifoldDoubleData* manifoldDataPtr)
{
	m_contactBreakingThreshold = manifoldDataPtr->m_contactBreakingThreshold;
	m_contactProcessingThreshold = manifoldDataPtr->m_contactProcessingThreshold;
	m_cachedPoints = manifoldDataPtr->m_numCachedPoints;
	m_companionIdA = manifoldDataPtr->m_companionIdA;
	m_companionIdB = manifoldDataPtr->m_companionIdB;
	//m_index1a = manifoldDataPtr->m_index1a;
	m_objectType = manifoldDataPtr->m_objectType;

	for (int i = 0; i < this->getNumContacts(); i++)
	{
		cbtManifoldPoint& pt = m_pointCache[i];

		pt.m_appliedImpulse = manifoldDataPtr->m_pointCacheAppliedImpulse[i];
		pt.m_appliedImpulseLateral1 = manifoldDataPtr->m_pointCacheAppliedImpulseLateral1[i];
		pt.m_appliedImpulseLateral2 = manifoldDataPtr->m_pointCacheAppliedImpulseLateral2[i];
		pt.m_localPointA.deSerializeDouble(manifoldDataPtr->m_pointCacheLocalPointA[i]);
		pt.m_localPointB.deSerializeDouble(manifoldDataPtr->m_pointCacheLocalPointB[i]);
		pt.m_normalWorldOnB.deSerializeDouble(manifoldDataPtr->m_pointCacheNormalWorldOnB[i]);
		pt.m_distance1 = manifoldDataPtr->m_pointCacheDistance[i];
		pt.m_combinedContactDamping1 = manifoldDataPtr->m_pointCacheCombinedContactDamping1[i];
		pt.m_combinedContactStiffness1 = manifoldDataPtr->m_pointCacheCombinedContactStiffness1[i];
		pt.m_lifeTime = manifoldDataPtr->m_pointCacheLifeTime[i];
		pt.m_frictionCFM = manifoldDataPtr->m_pointCacheFrictionCFM[i];
		pt.m_contactERP = manifoldDataPtr->m_pointCacheContactERP[i];
		pt.m_contactCFM = manifoldDataPtr->m_pointCacheContactCFM[i];
		pt.m_contactPointFlags = manifoldDataPtr->m_pointCacheContactPointFlags[i];
		pt.m_index0 = manifoldDataPtr->m_pointCacheIndex0[i];
		pt.m_index1 = manifoldDataPtr->m_pointCacheIndex1[i];
		pt.m_partId0 = manifoldDataPtr->m_pointCachePartId0[i];
		pt.m_partId1 = manifoldDataPtr->m_pointCachePartId1[i];
		pt.m_positionWorldOnA.deSerializeDouble(manifoldDataPtr->m_pointCachePositionWorldOnA[i]);
		pt.m_positionWorldOnB.deSerializeDouble(manifoldDataPtr->m_pointCachePositionWorldOnB[i]);
		pt.m_combinedFriction = manifoldDataPtr->m_pointCacheCombinedFriction[i];
		pt.m_lateralFrictionDir1.deSerializeDouble(manifoldDataPtr->m_pointCacheLateralFrictionDir1[i]);
		pt.m_lateralFrictionDir2.deSerializeDouble(manifoldDataPtr->m_pointCacheLateralFrictionDir2[i]);
		pt.m_combinedRollingFriction = manifoldDataPtr->m_pointCacheCombinedRollingFriction[i];
		pt.m_combinedSpinningFriction = manifoldDataPtr->m_pointCacheCombinedSpinningFriction[i];
		pt.m_combinedRestitution = manifoldDataPtr->m_pointCacheCombinedRestitution[i];
		pt.m_contactMotion1 = manifoldDataPtr->m_pointCacheContactMotion1[i];
		pt.m_contactMotion2 = manifoldDataPtr->m_pointCacheContactMotion2[i];
	}
}

void cbtPersistentManifold::deSerialize(const struct cbtPersistentManifoldFloatData* manifoldDataPtr)
{
	m_contactBreakingThreshold = manifoldDataPtr->m_contactBreakingThreshold;
	m_contactProcessingThreshold = manifoldDataPtr->m_contactProcessingThreshold;
	m_cachedPoints = manifoldDataPtr->m_numCachedPoints;
	m_companionIdA = manifoldDataPtr->m_companionIdA;
	m_companionIdB = manifoldDataPtr->m_companionIdB;
	//m_index1a = manifoldDataPtr->m_index1a;
	m_objectType = manifoldDataPtr->m_objectType;

	for (int i = 0; i < this->getNumContacts(); i++)
	{
		cbtManifoldPoint& pt = m_pointCache[i];

		pt.m_appliedImpulse = manifoldDataPtr->m_pointCacheAppliedImpulse[i];
		pt.m_appliedImpulseLateral1 = manifoldDataPtr->m_pointCacheAppliedImpulseLateral1[i];
		pt.m_appliedImpulseLateral2 = manifoldDataPtr->m_pointCacheAppliedImpulseLateral2[i];
		pt.m_localPointA.deSerialize(manifoldDataPtr->m_pointCacheLocalPointA[i]);
		pt.m_localPointB.deSerialize(manifoldDataPtr->m_pointCacheLocalPointB[i]);
		pt.m_normalWorldOnB.deSerialize(manifoldDataPtr->m_pointCacheNormalWorldOnB[i]);
		pt.m_distance1 = manifoldDataPtr->m_pointCacheDistance[i];
		pt.m_combinedContactDamping1 = manifoldDataPtr->m_pointCacheCombinedContactDamping1[i];
		pt.m_combinedContactStiffness1 = manifoldDataPtr->m_pointCacheCombinedContactStiffness1[i];
		pt.m_lifeTime = manifoldDataPtr->m_pointCacheLifeTime[i];
		pt.m_frictionCFM = manifoldDataPtr->m_pointCacheFrictionCFM[i];
		pt.m_contactERP = manifoldDataPtr->m_pointCacheContactERP[i];
		pt.m_contactCFM = manifoldDataPtr->m_pointCacheContactCFM[i];
		pt.m_contactPointFlags = manifoldDataPtr->m_pointCacheContactPointFlags[i];
		pt.m_index0 = manifoldDataPtr->m_pointCacheIndex0[i];
		pt.m_index1 = manifoldDataPtr->m_pointCacheIndex1[i];
		pt.m_partId0 = manifoldDataPtr->m_pointCachePartId0[i];
		pt.m_partId1 = manifoldDataPtr->m_pointCachePartId1[i];
		pt.m_positionWorldOnA.deSerialize(manifoldDataPtr->m_pointCachePositionWorldOnA[i]);
		pt.m_positionWorldOnB.deSerialize(manifoldDataPtr->m_pointCachePositionWorldOnB[i]);
		pt.m_combinedFriction = manifoldDataPtr->m_pointCacheCombinedFriction[i];
		pt.m_lateralFrictionDir1.deSerialize(manifoldDataPtr->m_pointCacheLateralFrictionDir1[i]);
		pt.m_lateralFrictionDir2.deSerialize(manifoldDataPtr->m_pointCacheLateralFrictionDir2[i]);
		pt.m_combinedRollingFriction = manifoldDataPtr->m_pointCacheCombinedRollingFriction[i];
		pt.m_combinedSpinningFriction = manifoldDataPtr->m_pointCacheCombinedSpinningFriction[i];
		pt.m_combinedRestitution = manifoldDataPtr->m_pointCacheCombinedRestitution[i];
		pt.m_contactMotion1 = manifoldDataPtr->m_pointCacheContactMotion1[i];
		pt.m_contactMotion2 = manifoldDataPtr->m_pointCacheContactMotion2[i];
	}
}