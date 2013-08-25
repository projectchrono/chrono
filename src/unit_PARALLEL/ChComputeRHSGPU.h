#ifndef CHCOMPUTERHSGPU_H
#define CHCOMPUTERHSGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChDataManager.h"
#include "core/ChTimer.h"
#include "ChBaseGPU.h"
namespace chrono {
class ChApiGPU ChComputeRHSGPU: public ChBaseGPU {
	public:
		ChComputeRHSGPU() {
		}
		~ChComputeRHSGPU() {
		}
		void Setup();
		void host_RHS_contacts(int2 *ids, real *correction, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs);
		void host_RHS_bilaterals(int2 *ids, real *correction, bool * active, real3 *vel, real3 *omega, real3 *JXYZA, real3 *JXYZB, real3 *JUVWA, real3 *JUVWB, real *rhs);
		void host_bi(real *correction, real* compliance, real* bi);
		void ComputeRHS(ChGPUDataManager *data_container_);

		custom_vector<real> correction, bi;
		ChTimer<double> timer_rhs;
		double time_rhs;
	};

	}

#endif
