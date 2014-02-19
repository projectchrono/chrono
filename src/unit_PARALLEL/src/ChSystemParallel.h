#ifndef CH_SYSTEMPARALLEL_H
#define CH_SYSTEMPARALLEL_H
//////////////////////////////////////////////////
//
//   ChSystemParallel.h
//
//   GPU Simulation System
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include <stdlib.h>
#include <float.h>
#include <memory.h>
#include <algorithm>
#include "ChParallelDefines.h"
#include "ChDataManager.h"
#include "core/ChTimer.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChBodyDEM.h"
#include "physics/ChGlobal.h"
#include "physics/ChContactContainer.h"
#include "ChLcpSolverParallel.h"
#include "ChLcpSystemDescriptorParallel.h"
#include "collision/ChCCollisionSystemParallel.h"
#include "collision/ChCCollisionSystemBulletParallel.h"
#include "math/ChParallelMath.h"

namespace chrono {

class ChApiGPU ChSystemParallel: public ChSystem {
	CH_RTTI(ChSystemParallel, ChSystem);

public:
	ChSystemParallel(unsigned int max_objects);

	virtual int Integrate_Y();

	void AddBody(ChSharedPtr<ChBody> newbody);
	virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody) = 0;
	void RemoveBody(ChSharedPtr<ChBody> mbody);
	void RemoveBody(int i);
	void Update();
	virtual void UpdateBodies() = 0;
	void UpdateBilaterals();
	void RecomputeThreads();
	void RecomputeBins();
	void PerturbBins(bool increase, int number = 2);

	virtual void ChangeCollisionSystem(ChCollisionSystem *newcollsystem);

	int GetNcontacts() {
		return gpu_data_manager->number_of_rigid_rigid;
	}

	void SetAABB(real3 aabbmin, real3 aabbmax) {
		aabb_min = aabbmin;
		aabb_max = aabbmax;
		use_aabb_active = true;
	}

	bool GetAABB(real3 &aabbmin, real3 &aabbmax) {
		aabbmin = aabb_min;
		aabbmax = aabb_max;

		return use_aabb_active;
	}

	double GetTimerCollision() {
		return timer_collision;
	}

	ChParallelDataManager *gpu_data_manager;

private:
	ChTimer<double> mtimer_lcp, mtimer_step, mtimer_cd_broad, mtimer_cd_narrow, mtimer_cd, mtimer_updt;

	unsigned int counter;
	double timer_collision;
	std::list<ChLink *>::iterator it;

	bool use_aabb_active;
	real3 aabb_min, aabb_max;

	int max_threads, current_threads, min_threads;
	vector<double> timer_accumulator, cd_accumulator;
	double old_timer, old_timer_cd;
	bool detect_optimal_threads;
	int detect_optimal_bins;
	uint frame_threads;
	uint frame_bins;
};


class ChApiGPU ChSystemParallelDVI : public ChSystemParallel {
	CH_RTTI(ChSystemParallelDVI, ChSystemParallel);

public:
	ChSystemParallelDVI(unsigned int max_objects = 1000);

	virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
	virtual void UpdateBodies();

};


class ChApiGPU ChSystemParallelDEM : public ChSystemParallel {
	CH_RTTI(ChSystemParallelDEM, ChSystemParallel);

public:
	ChSystemParallelDEM(unsigned int max_objects = 1000);

	virtual void LoadMaterialSurfaceData(ChSharedPtr<ChBody> newbody);
	virtual void UpdateBodies();

	virtual void ChangeCollisionSystem(ChCollisionSystem *newcollsystem);
};


}  // end namespace chrono



#endif

