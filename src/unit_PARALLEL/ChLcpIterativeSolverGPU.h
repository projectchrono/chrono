#ifndef CHC_LCPITERATIVESOLVERGPU_H
#define CHC_LCPITERATIVESOLVERGPU_H
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "ChCuda.h"
#include <cutil_math.h>
#include "ChGPUDataManager.h"
#define HM_REAL float
#define HM_REAL3 float3
#define HM_REAL4 float4

namespace chrono {
	class ChApiGPU ChLcpIterativeSolverGPU {
		public:
			//ChLcpIterativeSolverGPU();
			~ChLcpIterativeSolverGPU();
			void CPU_Version() {
			}
			;
			void Warm_Start();
			void RunTimeStep();

			float negated_recovery_speed, c_factor, step_size, tolerance, lcp_omega_bilateral, lcp_omega_contact, force_factor;
			uint number_of_contacts, number_of_bilaterals, number_of_constraints, number_of_bodies, number_of_updates, iteration_number, maximum_iterations;
			bool use_DEM;
			cudaEvent_t start, stop;

			ChGPUDataManager * data_container;

			thrust::host_vector<float4> host_bilateral_data;

		private:

			thrust::device_vector<uint> body_number;
			thrust::device_vector<uint> update_number;
			thrust::device_vector<uint> offset_counter;
			thrust::device_vector<uint> update_offset;
			thrust::device_vector<float> delta_gamma;
			thrust::device_vector<updateGPU> iteration_update;

			thrust::device_vector<float3> device_gamma;
			thrust::device_vector<float> device_deltagamma;
			thrust::device_vector<float4> device_bilateral_data;

			thrust::device_vector<int2> 		device_bids_data_old;

			//thrust::host_vector<contactGPU> host_contact_data;

	};

}
#endif

