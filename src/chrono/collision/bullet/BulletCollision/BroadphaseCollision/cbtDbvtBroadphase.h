/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///cbtDbvtBroadphase implementation by Nathanael Presson
#ifndef BT_DBVT_BROADPHASE_H
#define BT_DBVT_BROADPHASE_H

#include "BulletCollision/BroadphaseCollision/cbtDbvt.h"
#include "BulletCollision/BroadphaseCollision/cbtOverlappingPairCache.h"

//
// Compile time config
//

#define DBVT_BP_PROFILE 0
//#define DBVT_BP_SORTPAIRS				1
#define DBVT_BP_PREVENTFALSEUPDATE 0
#define DBVT_BP_ACCURATESLEEPING 0
#define DBVT_BP_ENABLE_BENCHMARK 0
//#define DBVT_BP_MARGIN				(cbtScalar)0.05 // ***CHRONO*** NO MARGIN
extern cbtScalar gDbvtMargin;

#if DBVT_BP_PROFILE
#define DBVT_BP_PROFILING_RATE 256
#include "LinearMath/cbtQuickprof.h"
#endif

//
// cbtDbvtProxy
//
struct cbtDbvtProxy : cbtBroadphaseProxy
{
	/* Fields		*/
	//cbtDbvtAabbMm	aabb;
	cbtDbvtNode* leaf;
	cbtDbvtProxy* links[2];
	int stage;
	/* ctor			*/
	cbtDbvtProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, void* userPtr, int collisionFilterGroup, int collisionFilterMask) : cbtBroadphaseProxy(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask)
	{
		links[0] = links[1] = 0;
	}
};

typedef cbtAlignedObjectArray<cbtDbvtProxy*> cbtDbvtProxyArray;

///The cbtDbvtBroadphase implements a broadphase using two dynamic AABB bounding volume hierarchies/trees (see cbtDbvt).
///One tree is used for static/non-moving objects, and another tree is used for dynamic objects. Objects can move from one tree to the other.
///This is a very fast broadphase, especially for very dynamic worlds where many objects are moving. Its insert/add and remove of objects is generally faster than the sweep and prune broadphases cbtAxisSweep3 and bt32BitAxisSweep3.
struct cbtDbvtBroadphase : cbtBroadphaseInterface
{
	/* Config		*/
	enum
	{
		DYNAMIC_SET = 0, /* Dynamic set index	*/
		FIXED_SET = 1,   /* Fixed set index		*/
		STAGECOUNT = 2   /* Number of stages		*/
	};
	/* Fields		*/
	cbtDbvt m_sets[2];                           // Dbvt sets
	cbtDbvtProxy* m_stageRoots[STAGECOUNT + 1];  // Stages list
	cbtOverlappingPairCache* m_paircache;        // Pair cache
	cbtScalar m_prediction;                      // Velocity prediction
	int m_stageCurrent;                         // Current stage
	int m_fupdates;                             // % of fixed updates per frame
	int m_dupdates;                             // % of dynamic updates per frame
	int m_cupdates;                             // % of cleanup updates per frame
	int m_newpairs;                             // Number of pairs created
	int m_fixedleft;                            // Fixed optimization left
	unsigned m_updates_call;                    // Number of updates call
	unsigned m_updates_done;                    // Number of updates done
	cbtScalar m_updates_ratio;                   // m_updates_done/m_updates_call
	int m_pid;                                  // Parse id
	int m_cid;                                  // Cleanup index
	int m_gid;                                  // Gen id
	bool m_releasepaircache;                    // Release pair cache on delete
	bool m_deferedcollide;                      // Defere dynamic/static collision to collide call
	bool m_needcleanup;                         // Need to run cleanup?
	cbtAlignedObjectArray<cbtAlignedObjectArray<const cbtDbvtNode*> > m_rayTestStacks;
#if DBVT_BP_PROFILE
	cbtClock m_clock;
	struct
	{
		unsigned long m_total;
		unsigned long m_ddcollide;
		unsigned long m_fdcollide;
		unsigned long m_cleanup;
		unsigned long m_jobcount;
	} m_profiling;
#endif
	/* Methods		*/
	cbtDbvtBroadphase(cbtOverlappingPairCache* paircache = 0);
	~cbtDbvtBroadphase();
	void collide(cbtDispatcher* dispatcher);
	void optimize();

	/* cbtBroadphaseInterface Implementation	*/
	cbtBroadphaseProxy* createProxy(const cbtVector3& aabbMin, const cbtVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, cbtDispatcher* dispatcher);
	virtual void destroyProxy(cbtBroadphaseProxy* proxy, cbtDispatcher* dispatcher);
	virtual void setAabb(cbtBroadphaseProxy* proxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* dispatcher);
	virtual void rayTest(const cbtVector3& rayFrom, const cbtVector3& rayTo, cbtBroadphaseRayCallback& rayCallback, const cbtVector3& aabbMin = cbtVector3(0, 0, 0), const cbtVector3& aabbMax = cbtVector3(0, 0, 0));
	virtual void aabbTest(const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtBroadphaseAabbCallback& callback);

	virtual void getAabb(cbtBroadphaseProxy* proxy, cbtVector3& aabbMin, cbtVector3& aabbMax) const;
	virtual void calculateOverlappingPairs(cbtDispatcher* dispatcher);
	virtual cbtOverlappingPairCache* getOverlappingPairCache();
	virtual const cbtOverlappingPairCache* getOverlappingPairCache() const;
	virtual void getBroadphaseAabb(cbtVector3& aabbMin, cbtVector3& aabbMax) const;
	virtual void printStats();

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(cbtDispatcher* dispatcher);

	void performDeferredRemoval(cbtDispatcher* dispatcher);

	void setVelocityPrediction(cbtScalar prediction)
	{
		m_prediction = prediction;
	}
	cbtScalar getVelocityPrediction() const
	{
		return m_prediction;
	}

	///this setAabbForceUpdate is similar to setAabb but always forces the aabb update.
	///it is not part of the cbtBroadphaseInterface but specific to cbtDbvtBroadphase.
	///it bypasses certain optimizations that prevent aabb updates (when the aabb shrinks), see
	///http://code.google.com/p/bullet/issues/detail?id=223
	void setAabbForceUpdate(cbtBroadphaseProxy* absproxy, const cbtVector3& aabbMin, const cbtVector3& aabbMax, cbtDispatcher* /*dispatcher*/);

	static void benchmark(cbtBroadphaseInterface*);
};

#endif
