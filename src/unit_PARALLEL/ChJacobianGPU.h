#ifndef CHJACOBIANGPU_H
#define CHJACOBIANGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChDataManager.h"
#include "ChBaseGPU.h"
namespace chrono {
	class ChApiGPU ChJacobianGPU:public ChBaseGPU {
		public:

			ChJacobianGPU() {
			}
			void Setup();
			void ComputeJacobians(ChGPUDataManager *data_container_);
			void host_ContactJacobians(real3* norm, real3* ptA, real3* ptB, int2* ids, real4* rot, real3* pos, real3* JXYZA, real3* JXYZB, real3* JUVWA, real3* JUVWB);

	};

}

#endif
