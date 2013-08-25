#ifndef CHINTEGRATORGPU_H
#define CHINTEGRATORGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChDataManager.h"

namespace chrono {
	class ChApiGPU ChIntegratorGPU {
		public:

			ChIntegratorGPU() {
			}
			void IntegrateSemiImplicit(real step, device_container& gpu_data_);
			void host_Integrate_Timestep_Semi_Implicit(bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim);

		protected:
			device_container *gpu_data;
			real step_size;
			uint number_of_objects;
	};

}

#endif
