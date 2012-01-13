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
#define HM_REAL float
#define HM_REAL3 float3
#define HM_REAL4 float4

namespace chrono {
class ChApiGPU ChLcpIterativeGPU {
	public:
		ChLcpIterativeGPU() {
		}
		~ChLcpIterativeGPU() {
		}
		static void RunTimeStep(
								uint & maximum_iterations,
								uint & iteration_number,
								float step_size,
								float lcp_omega_bilateral,
								float lcp_omega_contact,
								float tolerance,
								gpu_container & gpu_data);

		static void RunTimeStep_HOST(
										uint & maximum_iterations,
										uint & iteration_number,
										float step_size,
										float lcp_omega_bilateral,
										float lcp_omega_contact,
										float tolerance,
										ChGPUDataManager * data_container);

		static void Preprocess(float step_size, gpu_container & gpu_data);
		static void Iterate(float step_size, float lcp_omega_bilateral, float lcp_omega_contact, gpu_container & gpu_data);
		static void Reduce(gpu_container & gpu_data);
		static void Integrate(float step_size, gpu_container & gpu_data);

		static void Preprocess_HOST(float step_size, ChGPUDataManager * data_container);
		static void Iterate_HOST(float step_size, float lcp_omega_bilateral, float lcp_omega_contact, ChGPUDataManager * data_container);
		static void Reduce_HOST(ChGPUDataManager * data_container){}
		static void Integrate_HOST(float step_size, ChGPUDataManager * data_container);

		void WarmContact(const int & i);
		static float Max_DeltaGamma(device_vector<float> &device_dgm_data);
		static float Min_DeltaGamma(device_vector<float> &device_dgm_data);
		static float Avg_DeltaGamma(uint number_of_constraints, device_vector<float> &device_dgm_data);

		static float Total_KineticEnergy(gpu_container & gpu_data);

		static float Total_KineticEnergy_HOST(ChGPUDataManager * data_container);

		cudaEvent_t start, stop;

};
}
#endif
