///////////////////////////////////////////////////
//
//   ChLcpIterativeSolverGPU.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <cutil.h>
#include <cuda_runtime_api.h>
#include "ChLcpIterativeSolverGPU.h"
#include "ChBodyGPU.h"
#include "ChCuda.h"
#include "physics/ChBody.h"
#include "physics/ChSystem.h"
#include "ChDataManager.h"

// Forward declarations
namespace chrono {
	ChLcpIterativeSolverGPUsimple::ChLcpIterativeSolverGPUsimple(
			ChContactContainerGPUsimple* container)
	{
		gpu_contact_container = container;
		gpu_solver = new ChLcpIterativeGPU();
		number_of_bodies = 0;
		mTolerance = 1e-5;
		mDt = .01;
		mMaxIterations = 100;
		mOmegaContact = .2;
		mOmegaBilateral = .2;
	}

	ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple()
	{
	}

	void ChLcpIterativeSolverGPUsimple::SolveSys(
			uint &number_of_objects,
			uint &number_of_bilaterals,
			uint &number_of_contacts,
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
			device_vector<float3> &device_lim_data)
	{
		double maxrecspeed = gpu_contact_container->Get_load_max_recovery_speed();
		if (gpu_contact_container->Get_load_do_clamp() == false) {
			maxrecspeed = 10e25;
		}

		//gpu_solver->data_container = data_container;
		gpu_solver->RunTimeStep(
				number_of_objects,
				number_of_bilaterals,
				number_of_contacts,
				-maxrecspeed,
				mDt,
				mMaxIterations,
				iteration_number,
				1.0,
				mOmegaBilateral,
				mOmegaContact,
				mTolerance,
				device_norm_data,
				device_cpta_data,
				device_cptb_data,
				device_dpth_data,
				device_bids_data,
				device_bilateral_data,
				device_pos_data,
				device_rot_data,
				device_vel_data,
				device_omg_data,
				device_acc_data,
				device_inr_data,
				device_gyr_data,
				device_frc_data,
				device_trq_data,
				device_fap_data,
				device_gam_data,
				device_aux_data,
				device_ObA_data,
				device_ObB_data,
				device_ObC_data,
				device_ObR_data,
				device_typ_data,
				device_lim_data);

	}
} // END_OF_NAMESPACE____
