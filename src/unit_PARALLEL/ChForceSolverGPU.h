#ifndef CH_FORCESOLVERGPU_H
#define CH_FORCESOLVERGPU_H

#include "ChCuda.h"
#include "ChDataManager.h"
namespace chrono {
	class ChApiGPU ChForceSolverGPU {
		public:

			ChForceSolverGPU() {

			}
			~ChForceSolverGPU();
			void Init() {
			}
			void CD();
			void ComputeForces();
			float3 gravity;
			ChGPUDataManager *data_container;
	};
}
#endif
