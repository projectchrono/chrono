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
		~ChLcpIterativeGPU();
		static void RunTimeStep(
				uint &number_of_objects,
				uint &number_of_bilaterals,
				uint &number_of_contacts,
				float negated_recovery_speed,
				float step_size,
				uint maximum_iterations,
				uint & iteration_number,
				float force_factor,
				float lcp_omega_bilateral,
				float lcp_omega_contact,
				float tolerance,
				device_vector<float3> &device_norm_data,
				device_vector<float3> &device_cpta_data,
				device_vector<float3> &device_cptb_data,
				device_vector<float> &device_dpth_data,
				device_vector<int2> & device_bids_data,
				device_vector<float4> & device_bilateral_data,
				device_vector<float3> &device_pos_data,
				device_vector<float4> &device_rot_data,
				device_vector<float3> & device_vel_data,
				device_vector<float3> & device_omg_data,
				device_vector<float3> &device_acc_data,
				device_vector<float3> & device_inr_data,
				device_vector<float3> & device_gyr_data,
				device_vector<float3> & device_frc_data,
				device_vector<float3> & device_trq_data,
				device_vector<float3> & device_fap_data,
				device_vector<float3> &device_gam_data,
				device_vector<float3> & device_aux_data,
				device_vector<float3> &device_ObA_data,
				device_vector<float3> &device_ObB_data,
				device_vector<float3> &device_ObC_data,
				device_vector<float4> &device_ObR_data,
				device_vector<int3> &device_typ_data,
				device_vector<float3> &device_lim_data);
		void WarmContact(const int & i);
		static float Max_DeltaGamma(device_vector<float> &device_dgm_data);
		static float Min_DeltaGamma(device_vector<float> &device_dgm_data);
		static float Avg_DeltaGamma(uint number_of_constraints, device_vector<
				float> &device_dgm_data);

		static float Total_KineticEnergy(
				uint & number_of_bodies,
				device_vector<float3> &device_vel_data,
				device_vector<float3> &device_aux_data,
				device_vector<float> &device_dgm_data);

		cudaEvent_t start, stop;

};
}
#endif
