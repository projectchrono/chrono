#ifndef CH_SYSTEMGPU_H
#define CH_SYSTEMGPU_H
//////////////////////////////////////////////////
//
//   ChSystemMultiGPU.h
//
//   GPU Simulation System
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "ChCuda.h"
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>
#include <algorithm>

#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"
#include "physics/ChCollide.h"
#include "physics/ChContactContainer.h"
#include "physics/ChProximityContainerBase.h"

#include "ChLcpIterativeSolverGPU.h"
#include "ChLcpSystemDescriptorGPU.h"

#include "core/ChTimer.h"
#include "ChForceSystemGPU.h"
#include "ChDataManager.h"
#include "ChCCollisionSystemGPU.h"
#include "ChSubdomainGPU.h"

namespace chrono {
using namespace chrono;

class ChApiGPU ChSystemMultiGPU: public ChSystem {
	CH_RTTI(ChSystemMultiGPU,ChObj)
		;

	public:
		ChSystemMultiGPU(unsigned int max_objects = 1000);
		virtual int Integrate_Y_impulse_Anitescu();
		double ComputeCollisions();
		double SolveSystem();
		double SplitData();
		void AddBody(ChSharedPtr<ChBodyGPU> newbody);
		void RemoveBody(ChSharedPtr<ChBodyGPU> mbody);

		void Update();
		void SetBounds(float3 min, float3 max) {
			bounding_min = min;
			bounding_max = max;
		}
		void ChangeCollisionSystem(ChCollisionSystem* newcollsystem);
		void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);

		void Compute_AABBs();

		float GetKineticEnergy() {
			//return ((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->Total_KineticEnergy();
			return 0;
		}

		ChGPUDataManager *gpu_data_manager;
		vector<ChSubdomainGPU> gpu_subdomains;
		float3 subdomain_dim;
	private:
		ChTimer<double> mtimer_lcp, mtimer_step, mtimer_cd;
		unsigned int counter;
		unsigned int max_obj;
		unsigned int num_gpu;
		unsigned int num_subdiv;
		uint3 slices;
		float3 bounding_min, bounding_max;
};
}

#endif
