#ifndef CH_SYSTEMGPU_H
#define CH_SYSTEMGPU_H
//////////////////////////////////////////////////
//
//   ChSystemGPU.h
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
#include "ChDataManager.h"
#include "ChCCollisionSystemGPU.h"

namespace chrono {
using namespace chrono;

class ChApiGPU ChSystemGPU: public ChSystem {
	CH_RTTI(ChSystemGPU,ChObj)
		;

	public:
		ChSystemGPU(unsigned int max_objects = 1000);
		virtual int Integrate_Y_impulse_Anitescu();
		double ComputeCollisions();
		double SolveSystem();
		double SplitData();
		void AddBody(ChSharedPtr<ChBodyGPU> newbody);
		void RemoveBody(ChSharedPtr<ChBodyGPU> mbody);

		void Update();
		void ChangeCollisionSystem(ChCollisionSystem* newcollsystem);
		void ChangeLcpSolverSpeed(ChLcpSolver* newsolver);

		float GetKineticEnergy() {
			if (use_cpu == false) {
				return ((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->Total_KineticEnergy(gpu_data_manager->gpu_data);
			} else {
				return ((ChLcpIterativeSolverGPUsimple*) (LCP_solver_speed))->Total_KineticEnergy_HOST(gpu_data_manager);
			}
		}
		void SetUseCPU(bool usecpu) {
			use_cpu = usecpu;
		}
		ChGPUDataManager *gpu_data_manager;
	private:
		ChTimer<double> mtimer_lcp, mtimer_step, mtimer_cd;
		unsigned int counter;
		unsigned int max_obj;
		unsigned int num_gpu;
		float3 bounding_min, bounding_max;
		bool use_cpu;
};
}

#endif
