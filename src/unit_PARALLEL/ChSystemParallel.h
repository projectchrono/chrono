#ifndef CH_SYSTEMGPU_H
#define CH_SYSTEMGPU_H
//////////////////////////////////////////////////
//
//   ChSystemGPU.h
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
#include "ChParallelMath.h"
#include "ChParallelDefines.h"
#include <stdlib.h>
#include <float.h>
#include <memory.h>
#include <algorithm>
#include "core/ChTimer.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChGlobal.h"
#include "physics/ChContactContainer.h"
#include "ChLcpSolverParallel.h"
#include "ChLcpSystemDescriptorParallel.h"
#include "ChDataManager.h"
#include "collision/ChCCollisionSystemParallel.h"
#include "collision/ChCCollisionSystemBulletParallel.h"
namespace chrono {
using namespace chrono;

class ChApiGPU ChSystemParallel: public ChSystem {
	CH_RTTI(ChSystemParallel, ChObj)
		;

	public:
		ChSystemParallel(unsigned int max_objects = 1000);
		virtual int Integrate_Y_impulse_Anitescu();
		double ComputeCollisions();
		double SolveSystem();
		double SplitData();
		void AddBody(ChSharedPtr<ChBody> newbody);
		void RemoveBody(ChSharedPtr<ChBody> mbody);
		void RemoveBody(int i);
		//int Setup();
		void Update();
		void UpdateBodies();
		void UpdateBilaterals();
		void ChangeCollisionSystem(ChCollisionSystem *newcollsystem);
		void ChangeLcpSystemDescriptor(ChLcpSystemDescriptor* newdescriptor);
		void ChangeLcpSolverSpeed(ChLcpSolver *newsolver);
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
		ChGPUDataManager *gpu_data_manager;
	private:
		ChTimer<double> mtimer_lcp, mtimer_step, mtimer_cd_broad, mtimer_cd_narrow, mtimer_cd, mtimer_updt;
		unsigned int counter;
		double timer_collision;
		std::list<ChLink *>::iterator it;

		bool use_aabb_active;
		real3 aabb_min, aabb_max;

};
}

#endif

