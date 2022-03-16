//Bullet Continuous Collision Detection and Physics Library
//Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

//
// cbtAxisSweep3.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.

#ifndef BT_AXIS_SWEEP_3_INTERNAL_H
#define BT_AXIS_SWEEP_3_INTERNAL_H

#include "LinearMath/cbtVector3.h"
#include "cbtOverlappingPairCache.h"
#include "cbtBroadphaseInterface.h"
#include "cbtBroadphaseProxy.h"
#include "cbtOverlappingPairCallback.h"
#include "cbtDbvtBroadphase.h"

//#define DEBUG_BROADPHASE 1
#define USE_OVERLAP_TEST_ON_REMOVES 1

/// The internal templace class cbtAxisSweep3Internal implements the sweep and prune broadphase.
/// It uses quantized integers to represent the begin and end points for each of the 3 axis.
/// Dont use this class directly, use cbtAxisSweep3 or bt32BitAxisSweep3 instead.
template <typename BP_FP_INT_TYPE>
class cbtAxisSweep3Internal : public cbtBroadphaseInterface
{
protected:
	BP_FP_INT_TYPE m_bpHandleMask;
	BP_FP_INT_TYPE m_handleSentinel;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	class Edge
	{
	public:
		BP_FP_INT_TYPE m_pos;  // low bit is min/max
		BP_FP_INT_TYPE m_handle;

		BP_FP_INT_TYPE IsMax() const { return static_cast<BP_FP_INT_TYPE>(m_pos & 1); }
	};

public:
	class Handle : public cbtBroadphaseProxy
	{
	public:
		BT_DECLARE_ALIGNED_ALLOCATOR();

		// indexes into the edge arrays
		BP_FP_INT_TYPE m_minEdges[3], m_maxEdges[3];  // 6 * 2 = 12
													  //		BP_FP_INT_TYPE m_uniqueId;
		cbtBroadphaseProxy* m_dbvtProxy;               //for faster raycast
		//void* m_pOwner; this is now in cbtBroadphaseProxy.m_clientObject

		SIMD_FORCE_INLINE void SetNextFree(BP_FP_INT_TYPE next) { m_minEdges[0] = next; }
		SIMD_FORCE_INLINE BP_FP_INT_TYPE GetNextFree() const { return m_minEdges[0]; }
	};  // 24 bytes + 24 for Edge structures = 44 bytes total per entry

protected:
	cbtVector3 m_worldAabbMin;  // overall system bounds
	cbtVector3 m_worldAabbMax;  // overall system bounds

	cbtVector3 m_quantize;  // scaling factor for quantization

	BP_FP_INT_TYPE m_numHandles;  // number of active handles
	BP_FP_INT_TYPE m_maxHandles;  // max number of handles
	Handle* m_pHandles;           // handles pool

	BP_FP_INT_TYPE m_firstFreeHandle;  // free handles list

	Edge* m_pEdges[3];  // edge arrays for the 3 axes (each array has m_maxHandles * 2 + 2 sentinel entries)
	void* m_pEdgesRawPtr[3];

	cbtOverlappingPairCache* m_pairCache;

	///cbtOverlappingPairCallback is an additional optional user callback for adding/removing overlapping pairs, similar interface to cbtOverlappingPairCache.
	cbtOverlappingPairCallback* m_userPairCallback;

	bool m_ownsPairCache;

	int m_invalidPair;

	///additional dynamic aabb structure, used to accelerate ray cast queries.
	///can be disabled using a optional argument in the constructor
	cbtDbvtBroadphase* m_raycastAccelerator;
	cbtOverlappingPairCache* m_nullPairCache;

	// allocation/deallocation
	BP_FP_INT_TYPE allocHandle();
	void freeHandle(BP_FP_INT_TYPE handle);

	bool testOverlap2D(const Handle* pHandleA, const Handle* pHandleB, int axis0, int axis1);

#ifdef DEBUG_BROADPHASE
	void debugPrintAxis(int axis, bool checkCardinality = true);
#endif  //DEBUG_BROADPHASE

	//Overlap* AddOverlap(BP_FP_INT_TYPE handleA, BP_FP_INT_TYPE handleB);
	//void RemoveOverlap(BP_FP_INT_TYPE handleA, BP_FP_INT_TYPE handleB);

	void sortMinDown(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* dispatcher, bool updateOverlaps);
	void sortMinUp(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* dispatcher, bool updateOverlaps);
	void sortMaxDown(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* dispatcher, bool updateOverlaps);
	void sortMaxUp(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* dispatcher, bool updateOverlaps);

public:
	cbtAxisSweep3Internal(const cbtVector3& worldAabbMin, const cbtVector3& worldAabbMax, BP_FP_INT_TYPE handleMask, BP_FP_INT_TYPE handleSentinel, BP_FP_INT_TYPE maxHandles = 16384, cbtOverlappingPairCache* pairCache = 0, bool disableRaycastAccelerator = false);

	virtual ~cbtAxisSweep3Internal();

	BP_FP_INT_TYPE getNumHandles() const
	{
		return m_numHandles;
	}

	virtual void calculateOverlappingPairs(cbtDispatcher* dispatcher);

	BP_FP_INT_TYPE addHandle(const cbtVector3& aabbMin, const cbtVector3& aabbMax, void* pOwner, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher);
	void removeHandle(BP_FP_INT_TYPE handle, cbtDispatcher* dispatcher);
	void updateHandle(BP_FP_INT_TYPE handle, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher);
	SIMD_FORCE_INLINE Handle* getHandle(BP_FP_INT_TYPE index) const { return m_pHandles + index; }

	virtual void resetPool(cbtDispatcher* dispatcher);

	void processAllOverlappingPairs(cbtOverlapCallback* callback);

	//Broadphase Interface
	virtual cbtBroadphaseProxy* createProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher);
	virtual void destroyProxy(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher);
	virtual void setAabb(cbtBroadphaseProxy* proxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher);
	virtual void getAabb(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	virtual void rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtBroadphaseRayCallback& rayCallback, const cbtVector3& aabbMin = cbtVector3(0, 0, 0), const cbtVector3& aabbMax = cbtVector3(0, 0, 0));
	virtual void aabbTest(const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtBroadphaseAabbCallback& callback);

	void quantize(BP_FP_INT_TYPE* out, const cbtVector3& point, int isMax) const;
	///unQuantize should be conservative: aabbMin/aabbMax should be larger then 'getAabb' result
	void unQuantize(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const;

	bool testAabbOverlap(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1);

	cbtOverlappingPairCache* getOverlappingPairCache()
	{
		return m_pairCache;
	}
	const cbtOverlappingPairCache* getOverlappingPairCache() const
	{
		return m_pairCache;
	}

	void setOverlappingPairUserCallback(cbtOverlappingPairCallback* pairCallback)
	{
		m_userPairCallback = pairCallback;
	}
	const cbtOverlappingPairCallback* getOverlappingPairUserCallback() const
	{
		return m_userPairCallback;
	}

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(cbtVector3& aabbMin, cbtVector3& aabbMax) const
	{
		aabbMin = m_worldAabbMin;
		aabbMax = m_worldAabbMax;
	}

	virtual void printStats()
	{
		/*		printf("cbtAxisSweep3.h\n");
		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
		printf("aabbMin=%f,%f,%f,aabbMax=%f,%f,%f\n",m_worldAabbMin.getX(),m_worldAabbMin.getY(),m_worldAabbMin.getZ(),
			m_worldAabbMax.getX(),m_worldAabbMax.getY(),m_worldAabbMax.getZ());
			*/
	}
};

////////////////////////////////////////////////////////////////////

#ifdef DEBUG_BROADPHASE
#include <stdio.h>

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3<BP_FP_INT_TYPE>::debugPrintAxis(int axis, bool checkCardinality)
{
	int numEdges = m_pHandles[0].m_maxEdges[axis];
	printf("SAP Axis %d, numEdges=%d\n", axis, numEdges);

	int i;
	for (i = 0; i < numEdges + 1; i++)
	{
		Edge* pEdge = m_pEdges[axis] + i;
		Handle* pHandlePrev = getHandle(pEdge->m_handle);
		int handleIndex = pEdge->IsMax() ? pHandlePrev->m_maxEdges[axis] : pHandlePrev->m_minEdges[axis];
		char beginOrEnd;
		beginOrEnd = pEdge->IsMax() ? 'E' : 'B';
		printf("	[%c,h=%d,p=%x,i=%d]\n", beginOrEnd, pEdge->m_handle, pEdge->m_pos, handleIndex);
	}

	if (checkCardinality)
		cbtAssert(numEdges == m_numHandles * 2 + 1);
}
#endif  //DEBUG_BROADPHASE

template <typename BP_FP_INT_TYPE>
cbtBroadphaseProxy* cbtAxisSweep3Internal<BP_FP_INT_TYPE>::createProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher)
{
	(void)shapeType;
	BP_FP_INT_TYPE handleId = addHandle(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher);

	Handle* handle = getHandle(handleId);

	if (m_raycastAccelerator)
	{
		cbtBroadphaseProxy* rayProxy = m_raycastAccelerator->createProxy(aabbMin, aabbMax, shapeType, userPtr, collisionFilterGroup, collisionFilterMask, dispatcher);
		handle->m_dbvtProxy = rayProxy;
	}
	return handle;
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::destroyProxy(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher)
{
	Handle* handle = static_cast<Handle*>(proxy);
	if (m_raycastAccelerator)
		m_raycastAccelerator->destroyProxy(handle->m_dbvtProxy, dispatcher);
	removeHandle(static_cast<BP_FP_INT_TYPE>(handle->m_uniqueId), dispatcher);
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::setAabb(cbtBroadphaseProxy* proxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher)
{
	Handle* handle = static_cast<Handle*>(proxy);
	handle->m_aabbMin = aabbMin;
	handle->m_aabbMax = aabbMax;
	updateHandle(static_cast<BP_FP_INT_TYPE>(handle->m_uniqueId), aabbMin, aabbMax, dispatcher);
	if (m_raycastAccelerator)
		m_raycastAccelerator->setAabb(handle->m_dbvtProxy, aabbMin, aabbMax, dispatcher);
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtBroadphaseRayCallback& rayCallback, const cbtVector3& aabbMin, const cbtVector3& aabbMax)
{
	if (m_raycastAccelerator)
	{
		m_raycastAccelerator->rayTest(rayFrom, rayTo, rayCallback, aabbMin, aabbMax);
	}
	else
	{
		//choose axis?
		BP_FP_INT_TYPE axis = 0;
		//for each proxy
		for (BP_FP_INT_TYPE i = 1; i < m_numHandles * 2 + 1; i++)
		{
			if (m_pEdges[axis][i].IsMax())
			{
				rayCallback.process(getHandle(m_pEdges[axis][i].m_handle));
			}
		}
	}
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::aabbTest(const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtBroadphaseAabbCallback& callback)
{
	if (m_raycastAccelerator)
	{
		m_raycastAccelerator->aabbTest(aabbMin, aabbMax, callback);
	}
	else
	{
		//choose axis?
		BP_FP_INT_TYPE axis = 0;
		//for each proxy
		for (BP_FP_INT_TYPE i = 1; i < m_numHandles * 2 + 1; i++)
		{
			if (m_pEdges[axis][i].IsMax())
			{
				Handle* handle = getHandle(m_pEdges[axis][i].m_handle);
				if (TestAabbAgainstAabb2(aabbMin, aabbMax, handle->m_aabbMin, handle->m_aabbMax))
				{
					callback.process(handle);
				}
			}
		}
	}
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::getAabb(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	Handle* pHandle = static_cast<Handle*>(proxy);
	aabbMin = pHandle->m_aabbMin;
	aabbMax = pHandle->m_aabbMax;
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::unQuantize(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const
{
	Handle* pHandle = static_cast<Handle*>(proxy);

	unsigned short vecInMin[3];
	unsigned short vecInMax[3];

	vecInMin[0] = m_pEdges[0][pHandle->m_minEdges[0]].m_pos;
	vecInMax[0] = m_pEdges[0][pHandle->m_maxEdges[0]].m_pos + 1;
	vecInMin[1] = m_pEdges[1][pHandle->m_minEdges[1]].m_pos;
	vecInMax[1] = m_pEdges[1][pHandle->m_maxEdges[1]].m_pos + 1;
	vecInMin[2] = m_pEdges[2][pHandle->m_minEdges[2]].m_pos;
	vecInMax[2] = m_pEdges[2][pHandle->m_maxEdges[2]].m_pos + 1;

	aabbMin.setValue((cbtScalar)(vecInMin[0]) / (m_quantize.getX()), (cbtScalar)(vecInMin[1]) / (m_quantize.getY()), (cbtScalar)(vecInMin[2]) / (m_quantize.getZ()));
	aabbMin += m_worldAabbMin;

	aabbMax.setValue((cbtScalar)(vecInMax[0]) / (m_quantize.getX()), (cbtScalar)(vecInMax[1]) / (m_quantize.getY()), (cbtScalar)(vecInMax[2]) / (m_quantize.getZ()));
	aabbMax += m_worldAabbMin;
}

template <typename BP_FP_INT_TYPE>
cbtAxisSweep3Internal<BP_FP_INT_TYPE>::cbtAxisSweep3Internal(const cbtVector3& worldAabbMin, const cbtVector3& worldAabbMax, BP_FP_INT_TYPE handleMask, BP_FP_INT_TYPE handleSentinel, BP_FP_INT_TYPE userMaxHandles, cbtOverlappingPairCache* pairCache, bool disableRaycastAccelerator)
	: m_bpHandleMask(handleMask),
	  m_handleSentinel(handleSentinel),
	  m_pairCache(pairCache),
	  m_userPairCallback(0),
	  m_ownsPairCache(false),
	  m_invalidPair(0),
	  m_raycastAccelerator(0)
{
	BP_FP_INT_TYPE maxHandles = static_cast<BP_FP_INT_TYPE>(userMaxHandles + 1);  //need to add one sentinel handle

	if (!m_pairCache)
	{
		void* ptr = cbtAlignedAlloc(sizeof(cbtHashedOverlappingPairCache), 16);
		m_pairCache = new (ptr) cbtHashedOverlappingPairCache();
		m_ownsPairCache = true;
	}

	if (!disableRaycastAccelerator)
	{
		m_nullPairCache = new (cbtAlignedAlloc(sizeof(cbtNullPairCache), 16)) cbtNullPairCache();
		m_raycastAccelerator = new (cbtAlignedAlloc(sizeof(cbtDbvtBroadphase), 16)) cbtDbvtBroadphase(m_nullPairCache);  //m_pairCache);
		m_raycastAccelerator->m_deferedcollide = true;                                                                //don't add/remove pairs
	}

	//cbtAssert(bounds.HasVolume());

	// init bounds
	m_worldAabbMin = worldAabbMin;
	m_worldAabbMax = worldAabbMax;

	cbtVector3 aabbSize = m_worldAabbMax - m_worldAabbMin;

	BP_FP_INT_TYPE maxInt = m_handleSentinel;

	m_quantize = cbtVector3(cbtScalar(maxInt), cbtScalar(maxInt), cbtScalar(maxInt)) / aabbSize;

	// allocate handles buffer, using cbtAlignedAlloc, and put all handles on free list
	m_pHandles = new Handle[maxHandles];

	m_maxHandles = maxHandles;
	m_numHandles = 0;

	// handle 0 is reserved as the null index, and is also used as the sentinel
	m_firstFreeHandle = 1;
	{
		for (BP_FP_INT_TYPE i = m_firstFreeHandle; i < maxHandles; i++)
			m_pHandles[i].SetNextFree(static_cast<BP_FP_INT_TYPE>(i + 1));
		m_pHandles[maxHandles - 1].SetNextFree(0);
	}

	{
		// allocate edge buffers
		for (int i = 0; i < 3; i++)
		{
			m_pEdgesRawPtr[i] = cbtAlignedAlloc(sizeof(Edge) * maxHandles * 2, 16);
			m_pEdges[i] = new (m_pEdgesRawPtr[i]) Edge[maxHandles * 2];
		}
	}
	//removed overlap management

	// make boundary sentinels

	m_pHandles[0].m_clientObject = 0;

	for (int axis = 0; axis < 3; axis++)
	{
		m_pHandles[0].m_minEdges[axis] = 0;
		m_pHandles[0].m_maxEdges[axis] = 1;

		m_pEdges[axis][0].m_pos = 0;
		m_pEdges[axis][0].m_handle = 0;
		m_pEdges[axis][1].m_pos = m_handleSentinel;
		m_pEdges[axis][1].m_handle = 0;
#ifdef DEBUG_BROADPHASE
		debugPrintAxis(axis);
#endif  //DEBUG_BROADPHASE
	}
}

template <typename BP_FP_INT_TYPE>
cbtAxisSweep3Internal<BP_FP_INT_TYPE>::~cbtAxisSweep3Internal()
{
	if (m_raycastAccelerator)
	{
		m_nullPairCache->~cbtOverlappingPairCache();
		cbtAlignedFree(m_nullPairCache);
		m_raycastAccelerator->~cbtDbvtBroadphase();
		cbtAlignedFree(m_raycastAccelerator);
	}

	for (int i = 2; i >= 0; i--)
	{
		cbtAlignedFree(m_pEdgesRawPtr[i]);
	}
	delete[] m_pHandles;

	if (m_ownsPairCache)
	{
		m_pairCache->~cbtOverlappingPairCache();
		cbtAlignedFree(m_pairCache);
	}
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::quantize(BP_FP_INT_TYPE* out, const cbtVector3& point, int isMax) const
{
#ifdef OLD_CLAMPING_METHOD
	///problem with this clamping method is that the floating point during quantization might still go outside the range [(0|isMax) .. (m_handleSentinel&m_bpHandleMask]|isMax]
	///see http://code.google.com/p/bullet/issues/detail?id=87
	cbtVector3 clampedPoint(point);
	clampedPoint.setMax(m_worldAabbMin);
	clampedPoint.setMin(m_worldAabbMax);
	cbtVector3 v = (clampedPoint - m_worldAabbMin) * m_quantize;
	out[0] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getX() & m_bpHandleMask) | isMax);
	out[1] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getY() & m_bpHandleMask) | isMax);
	out[2] = (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v.getZ() & m_bpHandleMask) | isMax);
#else
	cbtVector3 v = (point - m_worldAabbMin) * m_quantize;
	out[0] = (v[0] <= 0) ? (BP_FP_INT_TYPE)isMax : (v[0] >= m_handleSentinel) ? (BP_FP_INT_TYPE)((m_handleSentinel & m_bpHandleMask) | isMax) : (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[0] & m_bpHandleMask) | isMax);
	out[1] = (v[1] <= 0) ? (BP_FP_INT_TYPE)isMax : (v[1] >= m_handleSentinel) ? (BP_FP_INT_TYPE)((m_handleSentinel & m_bpHandleMask) | isMax) : (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[1] & m_bpHandleMask) | isMax);
	out[2] = (v[2] <= 0) ? (BP_FP_INT_TYPE)isMax : (v[2] >= m_handleSentinel) ? (BP_FP_INT_TYPE)((m_handleSentinel & m_bpHandleMask) | isMax) : (BP_FP_INT_TYPE)(((BP_FP_INT_TYPE)v[2] & m_bpHandleMask) | isMax);
#endif  //OLD_CLAMPING_METHOD
}

template <typename BP_FP_INT_TYPE>
BP_FP_INT_TYPE cbtAxisSweep3Internal<BP_FP_INT_TYPE>::allocHandle()
{
	cbtAssert(m_firstFreeHandle);

	BP_FP_INT_TYPE handle = m_firstFreeHandle;
	m_firstFreeHandle = getHandle(handle)->GetNextFree();
	m_numHandles++;

	return handle;
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::freeHandle(BP_FP_INT_TYPE handle)
{
	cbtAssert(handle > 0 && handle < m_maxHandles);

	getHandle(handle)->SetNextFree(m_firstFreeHandle);
	m_firstFreeHandle = handle;

	m_numHandles--;
}

template <typename BP_FP_INT_TYPE>
BP_FP_INT_TYPE cbtAxisSweep3Internal<BP_FP_INT_TYPE>::addHandle(const cbtVector3& aabbMin, const cbtVector3& aabbMax, void* pOwner, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher)
{
	// quantize the bounds
	BP_FP_INT_TYPE min[3], max[3];
	quantize(min, aabbMin, 0);
	quantize(max, aabbMax, 1);

	// allocate a handle
	BP_FP_INT_TYPE handle = allocHandle();

	Handle* pHandle = getHandle(handle);

	pHandle->m_uniqueId = static_cast<int>(handle);
	//pHandle->m_pOverlaps = 0;
	pHandle->m_clientObject = pOwner;
	pHandle->m_collisionFilterGroup = collisionFilterGroup;
	pHandle->m_collisionFilterMask = collisionFilterMask;

	// compute current limit of edge arrays
	BP_FP_INT_TYPE limit = static_cast<BP_FP_INT_TYPE>(m_numHandles * 2);

	// insert new edges just inside the max boundary edge
	for (BP_FP_INT_TYPE axis = 0; axis < 3; axis++)
	{
		m_pHandles[0].m_maxEdges[axis] += 2;

		m_pEdges[axis][limit + 1] = m_pEdges[axis][limit - 1];

		m_pEdges[axis][limit - 1].m_pos = min[axis];
		m_pEdges[axis][limit - 1].m_handle = handle;

		m_pEdges[axis][limit].m_pos = max[axis];
		m_pEdges[axis][limit].m_handle = handle;

		pHandle->m_minEdges[axis] = static_cast<BP_FP_INT_TYPE>(limit - 1);
		pHandle->m_maxEdges[axis] = limit;
	}

	// now sort the new edges to their correct position
	sortMinDown(0, pHandle->m_minEdges[0], dispatcher, false);
	sortMaxDown(0, pHandle->m_maxEdges[0], dispatcher, false);
	sortMinDown(1, pHandle->m_minEdges[1], dispatcher, false);
	sortMaxDown(1, pHandle->m_maxEdges[1], dispatcher, false);
	sortMinDown(2, pHandle->m_minEdges[2], dispatcher, true);
	sortMaxDown(2, pHandle->m_maxEdges[2], dispatcher, true);

	return handle;
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::removeHandle(BP_FP_INT_TYPE handle, cbtDispatcher* dispatcher)
{
	Handle* pHandle = getHandle(handle);

	//explicitly remove the pairs containing the proxy
	//we could do it also in the sortMinUp (passing true)
	///@todo: compare performance
	if (!m_pairCache->hasDeferredRemoval())
	{
		m_pairCache->removeOverlappingPairsContainingProxy(pHandle, dispatcher);
	}

	// compute current limit of edge arrays
	int limit = static_cast<int>(m_numHandles * 2);

	int axis;

	for (axis = 0; axis < 3; axis++)
	{
		m_pHandles[0].m_maxEdges[axis] -= 2;
	}

	// remove the edges by sorting them up to the end of the list
	for (axis = 0; axis < 3; axis++)
	{
		Edge* pEdges = m_pEdges[axis];
		BP_FP_INT_TYPE max = pHandle->m_maxEdges[axis];
		pEdges[max].m_pos = m_handleSentinel;

		sortMaxUp(axis, max, dispatcher, false);

		BP_FP_INT_TYPE i = pHandle->m_minEdges[axis];
		pEdges[i].m_pos = m_handleSentinel;

		sortMinUp(axis, i, dispatcher, false);

		pEdges[limit - 1].m_handle = 0;
		pEdges[limit - 1].m_pos = m_handleSentinel;

#ifdef DEBUG_BROADPHASE
		debugPrintAxis(axis, false);
#endif  //DEBUG_BROADPHASE
	}

	// free the handle
	freeHandle(handle);
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::resetPool(cbtDispatcher* /*dispatcher*/)
{
	if (m_numHandles == 0)
	{
		m_firstFreeHandle = 1;
		{
			for (BP_FP_INT_TYPE i = m_firstFreeHandle; i < m_maxHandles; i++)
				m_pHandles[i].SetNextFree(static_cast<BP_FP_INT_TYPE>(i + 1));
			m_pHandles[m_maxHandles - 1].SetNextFree(0);
		}
	}
}

//#include <stdio.h>

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::calculateOverlappingPairs(cbtDispatcher* dispatcher)
{
	if (m_pairCache->hasDeferredRemoval())
	{
		cbtBroadphasePairArray& overlappingPairArray = m_pairCache->getOverlappingPairArray();

		//perform a sort, to find duplicates and to sort 'invalid' pairs to the end
		overlappingPairArray.quickSort(cbtBroadphasePairSortPredicate());

		overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
		m_invalidPair = 0;

		int i;

		cbtBroadphasePair previousPair;
		previousPair.m_pProxy0 = 0;
		previousPair.m_pProxy1 = 0;
		previousPair.m_algorithm = 0;

		for (i = 0; i < overlappingPairArray.size(); i++)
		{
			cbtBroadphasePair& pair = overlappingPairArray[i];

			bool isDuplicate = (pair == previousPair);

			previousPair = pair;

			bool needsRemoval = false;

			if (!isDuplicate)
			{
				///important to use an AABB test that is consistent with the broadphase
				bool hasOverlap = testAabbOverlap(pair.m_pProxy0, pair.m_pProxy1);

				if (hasOverlap)
				{
					needsRemoval = false;  //callback->processOverlap(pair);
				}
				else
				{
					needsRemoval = true;
				}
			}
			else
			{
				//remove duplicate
				needsRemoval = true;
				//should have no algorithm
				cbtAssert(!pair.m_algorithm);
			}

			if (needsRemoval)
			{
				m_pairCache->cleanOverlappingPair(pair, dispatcher);

				//		m_overlappingPairArray.swap(i,m_overlappingPairArray.size()-1);
				//		m_overlappingPairArray.pop_back();
				pair.m_pProxy0 = 0;
				pair.m_pProxy1 = 0;
				m_invalidPair++;
			}
		}

///if you don't like to skip the invalid pairs in the array, execute following code:
#define CLEAN_INVALID_PAIRS 1
#ifdef CLEAN_INVALID_PAIRS

		//perform a sort, to sort 'invalid' pairs to the end
		overlappingPairArray.quickSort(cbtBroadphasePairSortPredicate());

		overlappingPairArray.resize(overlappingPairArray.size() - m_invalidPair);
		m_invalidPair = 0;
#endif  //CLEAN_INVALID_PAIRS

		//printf("overlappingPairArray.size()=%d\n",overlappingPairArray.size());
	}
}

template <typename BP_FP_INT_TYPE>
bool cbtAxisSweep3Internal<BP_FP_INT_TYPE>::testAabbOverlap(cbtBroadphaseProxy* proxy0, cbtBroadphaseProxy* proxy1)
{
	const Handle* pHandleA = static_cast<Handle*>(proxy0);
	const Handle* pHandleB = static_cast<Handle*>(proxy1);

	//optimization 1: check the array index (memory address), instead of the m_pos

	for (int axis = 0; axis < 3; axis++)
	{
		if (pHandleA->m_maxEdges[axis] < pHandleB->m_minEdges[axis] ||
			pHandleB->m_maxEdges[axis] < pHandleA->m_minEdges[axis])
		{
			return false;
		}
	}
	return true;
}

template <typename BP_FP_INT_TYPE>
bool cbtAxisSweep3Internal<BP_FP_INT_TYPE>::testOverlap2D(const Handle* pHandleA, const Handle* pHandleB, int axis0, int axis1)
{
	//optimization 1: check the array index (memory address), instead of the m_pos

	if (pHandleA->m_maxEdges[axis0] < pHandleB->m_minEdges[axis0] ||
		pHandleB->m_maxEdges[axis0] < pHandleA->m_minEdges[axis0] ||
		pHandleA->m_maxEdges[axis1] < pHandleB->m_minEdges[axis1] ||
		pHandleB->m_maxEdges[axis1] < pHandleA->m_minEdges[axis1])
	{
		return false;
	}
	return true;
}

template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::updateHandle(BP_FP_INT_TYPE handle, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher)
{
	//	cbtAssert(bounds.IsFinite());
	//cbtAssert(bounds.HasVolume());

	Handle* pHandle = getHandle(handle);

	// quantize the new bounds
	BP_FP_INT_TYPE min[3], max[3];
	quantize(min, aabbMin, 0);
	quantize(max, aabbMax, 1);

	// update changed edges
	for (int axis = 0; axis < 3; axis++)
	{
		BP_FP_INT_TYPE emin = pHandle->m_minEdges[axis];
		BP_FP_INT_TYPE emax = pHandle->m_maxEdges[axis];

		int dmin = (int)min[axis] - (int)m_pEdges[axis][emin].m_pos;
		int dmax = (int)max[axis] - (int)m_pEdges[axis][emax].m_pos;

		m_pEdges[axis][emin].m_pos = min[axis];
		m_pEdges[axis][emax].m_pos = max[axis];

		// expand (only adds overlaps)
		if (dmin < 0)
			sortMinDown(axis, emin, dispatcher, true);

		if (dmax > 0)
			sortMaxUp(axis, emax, dispatcher, true);

		// shrink (only removes overlaps)
		if (dmin > 0)
			sortMinUp(axis, emin, dispatcher, true);

		if (dmax < 0)
			sortMaxDown(axis, emax, dispatcher, true);

#ifdef DEBUG_BROADPHASE
		debugPrintAxis(axis);
#endif  //DEBUG_BROADPHASE
	}
}

// sorting a min edge downwards can only ever *add* overlaps
template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::sortMinDown(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* /* dispatcher */, bool updateOverlaps)
{
	Edge* pEdge = m_pEdges[axis] + edge;
	Edge* pPrev = pEdge - 1;
	Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pEdge->m_pos < pPrev->m_pos)
	{
		Handle* pHandlePrev = getHandle(pPrev->m_handle);

		if (pPrev->IsMax())
		{
			// if previous edge is a maximum check the bounds and add an overlap if necessary
			const int axis1 = (1 << axis) & 3;
			const int axis2 = (1 << axis1) & 3;
			if (updateOverlaps && testOverlap2D(pHandleEdge, pHandlePrev, axis1, axis2))
			{
				m_pairCache->addOverlappingPair(pHandleEdge, pHandlePrev);
				if (m_userPairCallback)
					m_userPairCallback->addOverlappingPair(pHandleEdge, pHandlePrev);

				//AddOverlap(pEdge->m_handle, pPrev->m_handle);
			}

			// update edge reference in other handle
			pHandlePrev->m_maxEdges[axis]++;
		}
		else
			pHandlePrev->m_minEdges[axis]++;

		pHandleEdge->m_minEdges[axis]--;

		// swap the edges
		Edge swap = *pEdge;
		*pEdge = *pPrev;
		*pPrev = swap;

		// decrement
		pEdge--;
		pPrev--;
	}

#ifdef DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif  //DEBUG_BROADPHASE
}

// sorting a min edge upwards can only ever *remove* overlaps
template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::sortMinUp(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* dispatcher, bool updateOverlaps)
{
	Edge* pEdge = m_pEdges[axis] + edge;
	Edge* pNext = pEdge + 1;
	Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pNext->m_handle && (pEdge->m_pos >= pNext->m_pos))
	{
		Handle* pHandleNext = getHandle(pNext->m_handle);

		if (pNext->IsMax())
		{
			Handle* handle0 = getHandle(pEdge->m_handle);
			Handle* handle1 = getHandle(pNext->m_handle);
			const int axis1 = (1 << axis) & 3;
			const int axis2 = (1 << axis1) & 3;

			// if next edge is maximum remove any overlap between the two handles
			if (updateOverlaps
#ifdef USE_OVERLAP_TEST_ON_REMOVES
				&& testOverlap2D(handle0, handle1, axis1, axis2)
#endif  //USE_OVERLAP_TEST_ON_REMOVES
			)
			{
				m_pairCache->removeOverlappingPair(handle0, handle1, dispatcher);
				if (m_userPairCallback)
					m_userPairCallback->removeOverlappingPair(handle0, handle1, dispatcher);
			}

			// update edge reference in other handle
			pHandleNext->m_maxEdges[axis]--;
		}
		else
			pHandleNext->m_minEdges[axis]--;

		pHandleEdge->m_minEdges[axis]++;

		// swap the edges
		Edge swap = *pEdge;
		*pEdge = *pNext;
		*pNext = swap;

		// increment
		pEdge++;
		pNext++;
	}
}

// sorting a max edge downwards can only ever *remove* overlaps
template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::sortMaxDown(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* dispatcher, bool updateOverlaps)
{
	Edge* pEdge = m_pEdges[axis] + edge;
	Edge* pPrev = pEdge - 1;
	Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pEdge->m_pos < pPrev->m_pos)
	{
		Handle* pHandlePrev = getHandle(pPrev->m_handle);

		if (!pPrev->IsMax())
		{
			// if previous edge was a minimum remove any overlap between the two handles
			Handle* handle0 = getHandle(pEdge->m_handle);
			Handle* handle1 = getHandle(pPrev->m_handle);
			const int axis1 = (1 << axis) & 3;
			const int axis2 = (1 << axis1) & 3;

			if (updateOverlaps
#ifdef USE_OVERLAP_TEST_ON_REMOVES
				&& testOverlap2D(handle0, handle1, axis1, axis2)
#endif  //USE_OVERLAP_TEST_ON_REMOVES
			)
			{
				//this is done during the overlappingpairarray iteration/narrowphase collision

				m_pairCache->removeOverlappingPair(handle0, handle1, dispatcher);
				if (m_userPairCallback)
					m_userPairCallback->removeOverlappingPair(handle0, handle1, dispatcher);
			}

			// update edge reference in other handle
			pHandlePrev->m_minEdges[axis]++;
			;
		}
		else
			pHandlePrev->m_maxEdges[axis]++;

		pHandleEdge->m_maxEdges[axis]--;

		// swap the edges
		Edge swap = *pEdge;
		*pEdge = *pPrev;
		*pPrev = swap;

		// decrement
		pEdge--;
		pPrev--;
	}

#ifdef DEBUG_BROADPHASE
	debugPrintAxis(axis);
#endif  //DEBUG_BROADPHASE
}

// sorting a max edge upwards can only ever *add* overlaps
template <typename BP_FP_INT_TYPE>
void cbtAxisSweep3Internal<BP_FP_INT_TYPE>::sortMaxUp(int axis, BP_FP_INT_TYPE edge, cbtDispatcher* /* dispatcher */, bool updateOverlaps)
{
	Edge* pEdge = m_pEdges[axis] + edge;
	Edge* pNext = pEdge + 1;
	Handle* pHandleEdge = getHandle(pEdge->m_handle);

	while (pNext->m_handle && (pEdge->m_pos >= pNext->m_pos))
	{
		Handle* pHandleNext = getHandle(pNext->m_handle);

		const int axis1 = (1 << axis) & 3;
		const int axis2 = (1 << axis1) & 3;

		if (!pNext->IsMax())
		{
			// if next edge is a minimum check the bounds and add an overlap if necessary
			if (updateOverlaps && testOverlap2D(pHandleEdge, pHandleNext, axis1, axis2))
			{
				Handle* handle0 = getHandle(pEdge->m_handle);
				Handle* handle1 = getHandle(pNext->m_handle);
				m_pairCache->addOverlappingPair(handle0, handle1);
				if (m_userPairCallback)
					m_userPairCallback->addOverlappingPair(handle0, handle1);
			}

			// update edge reference in other handle
			pHandleNext->m_minEdges[axis]--;
		}
		else
			pHandleNext->m_maxEdges[axis]--;

		pHandleEdge->m_maxEdges[axis]++;

		// swap the edges
		Edge swap = *pEdge;
		*pEdge = *pNext;
		*pNext = swap;

		// increment
		pEdge++;
		pNext++;
	}
}

#endif
