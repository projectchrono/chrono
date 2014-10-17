#ifndef CHINTEGRATORGPU_H
#define CHINTEGRATORGPU_H

#include "chrono_parallel/ChParallelDefines.h"
#include "chrono_parallel/ChDataManager.h"
#include "chrono_parallel/math/ChParallelMath.h"
#include "chrono_parallel/math/ChThrustLinearAlgebra.h"

namespace chrono {
	class CH_PARALLEL_API ChIntegratorParallel {
		public:

			ChIntegratorParallel() {
			}
			void IntegrateSemiImplicit(real step, host_container& gpu_data_);
			void host_Integrate_Timestep_Semi_Implicit(bool* active, real3* acc, real4* rot, real3* vel, real3* omega, real3* pos, real3* lim);

		protected:
			host_container *gpu_data;
			real step_size;
			uint number_of_objects;
	};

}

#endif
