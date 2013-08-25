#ifndef CHBASEGPU_H
#define CHBASEGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChDataManager.h"
#include "core/ChTimer.h"
namespace chrono {
class ChApiGPU ChBaseGPU {
	public:
		ChBaseGPU() {
		}
		~ChBaseGPU() {
		}

		void Initialize();
	protected:
		ChGPUDataManager *data_container;
		uint number_of_rigid;
		uint number_of_rigid_rigid;
		uint number_of_bilaterals;
		uint number_of_constraints;
		uint number_of_updates;

		real step_size;

		real inv_hpa;
		real inv_hhpa;

//		real compliance;
//		real complianceT;
		real alpha;
		real contact_recovery_speed;

};

}

#endif
