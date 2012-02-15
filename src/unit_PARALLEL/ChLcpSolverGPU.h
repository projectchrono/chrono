#ifndef CHC_LCPITERATIVESOLVERGPU_H
#define CHC_LCPITERATIVESOLVERGPU_H

//////////////////////////////////////////////////
//
//   ChIterativeGPU.h
//
//   GPU LCP Solver
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ChCuda.h"
#include <cutil_math.h>
#include "ChDataManager.h"
#include "lcp/ChLcpIterativeSolver.h"
#define HM_REAL float
#define HM_REAL3 float3
#define HM_REAL4 float4

namespace chrono {
	class ChApiGPU ChLcpSolverGPU: public ChLcpIterativeSolver {
		public:
			ChLcpSolverGPU() {
			}
			~ChLcpSolverGPU() {
			}
			virtual double Solve(ChLcpSystemDescriptor& sysd, bool add_Mq_to_f = false) {
				return 0;
			}

			void RunTimeStep(float step, gpu_container & gpu_data);
			void Preprocess(gpu_container & gpu_data);
			void Iterate(gpu_container & gpu_data);
			void Reduce(gpu_container & gpu_data);
			void Integrate(gpu_container & gpu_data);
			void WarmContact(const int & i);
			float Max_DeltaGamma(device_vector<float> &device_dgm_data);
			float Min_DeltaGamma(device_vector<float> &device_dgm_data);
			float Avg_DeltaGamma(uint number_of_constraints, device_vector<float> &device_dgm_data);
			float Total_KineticEnergy(gpu_container & gpu_data);

			uint GetIterations() {
				return iteration_number;
			}

		private:
			unsigned int iteration_number;
			float step_size;
			float lcp_omega_bilateral;
			float lcp_omega_contact;
			float compliance;
			float complianceT;
			float alpha;
			cudaEvent_t start, stop;
	};
}
#endif
