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
			ChLcpIterativeGPU(){
				compliance=1e-8;
				complianceT=4e-8;
				alpha=.2;
				use_DEM=false;
			}
			~ChLcpIterativeGPU();
			void RunTimeStep(const int & i);
			void WarmContact(const int & i);
			float Max_DeltaGamma();
			float Min_DeltaGamma();
			float Avg_DeltaGamma();

			float Total_KineticEnergy();

			float negated_recovery_speed, c_factor, step_size, tolerance, lcp_omega_bilateral, lcp_omega_contact, force_factor, compliance,complianceT, alpha;
			uint number_of_contacts, number_of_bilaterals, number_of_constraints, number_of_bodies, number_of_updates, iteration_number, maximum_iterations;
			bool use_DEM;
			cudaEvent_t start, stop;

			ChGPUDataManager * data_container;
		private:

			thrust::device_vector<uint> body_number;
			thrust::device_vector<uint> update_number;
			thrust::device_vector<uint> offset_counter;
			thrust::device_vector<uint> update_offset;
			thrust::device_vector<float> delta_gamma;
			thrust::device_vector<float3> vel_update;
			thrust::device_vector<float3> omg_update;
			thrust::device_vector<float> device_ken_data;
			thrust::device_vector<float> device_dgm_data;
			thrust::device_vector<float> device_ctd_data;

			//thrust::host_vector<contactGPU> host_contact_data;
	};
}
#endif
