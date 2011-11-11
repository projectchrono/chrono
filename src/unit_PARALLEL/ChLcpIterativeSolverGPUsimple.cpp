///////////////////////////////////////////////////
//
//   ChLcpIterativeSolverGPUsimple.cpp
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
#include "ChLcpIterativeSolverGPUsimple.h"
#include "ChLcpConstraintTwoGPUcontN.h"
#include "ChBodyGPU.h"
#include "ChCuda.h"
#include "physics/ChBody.h"
#include "physics/ChSystem.h"
#include "ChGPUDataManager.h"

// Forward declarations
namespace chrono {
	ChLcpIterativeSolverGPUsimple::ChLcpIterativeSolverGPUsimple(ChContactContainerGPUsimple* container, ChLcpSystemDescriptorGPU* descriptor) {
		gpu_contact_container = container;
		mSystemDescriptor = descriptor;
		gpu_solver = mSystemDescriptor->gpu_solver;

		number_of_bodies = 0;
		force_solver = new ChForceSolverGPU();

		mTolerance = 1e-5;
		mDt = .01;
		mMaxIterations = 100;
		mOmegaContact = .2;
		mOmegaBilateral = .2;

	}

	ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple() {
	}

	double ChLcpIterativeSolverGPUsimple::Solve(ChLcpSystemDescriptor& sysd, ///< system description with constraints and variables
			bool add_Mq_to_f) ///< if true, takes the initial 'q' and adds [M]*q to 'f' vector
	{
		std::vector<ChLcpConstraint*>& mconstraints = sysd.GetConstraintsList();
		std::vector<ChLcpVariables*>& mvariables = sysd.GetVariablesList();

		// -1-  Count active variables and initialize GPU body buffer
		//uint number_of_bodies = mvariables.size();


		mSystemDescriptor->number_of_bodies = number_of_bodies;

		// -2-  Count active constraints and initialize GPU bilateral buffer.

		unsigned int number_of_bilaterals = 0;
		for (uint i = 0; i < mconstraints.size(); i++) {
			if (mconstraints[i]->IsActive()) {
				number_of_bilaterals++;
			}
		}

		gpu_solver->host_bilateral_data.resize(number_of_bilaterals * CH_BILATERAL_VSIZE);
		uint counter = 0;
#pragma omp parallel for
		for (uint ic = 0; ic < mconstraints.size(); ic++) {
			if (!mconstraints[ic]->IsActive()) {
				continue;
			}
			ChLcpConstraintTwoBodies* mbilateral = (ChLcpConstraintTwoBodies*) (mconstraints[ic]);

			ChBodyGPU* temp = (ChBodyGPU*) (((ChLcpVariablesBody*) (mbilateral->GetVariables_a()))->GetUserData());

			int idA = ((ChBodyGPU*) ((ChLcpVariablesBody*) (mbilateral->GetVariables_a()))->GetUserData())->id;
			int idB = ((ChBodyGPU*) ((ChLcpVariablesBody*) (mbilateral->GetVariables_b()))->GetUserData())->id;

			// Update auxiliary data in all constraints before starting, that is: g_i=[Cq_i]*[invM_i]*[Cq_i]' and  [Eq_i]=[invM_i]*[Cq_i]'
			mconstraints[ic]->Update_auxiliary(); //***NOTE*** not efficient here - can be on GPU, and [Eq_i] not needed
			float4 A, B, C, D;
			A = F4(mbilateral->Get_Cq_a()->GetElementN(0), mbilateral->Get_Cq_a()->GetElementN(1), mbilateral->Get_Cq_a()->GetElementN(2), 0);//J1x
			B = F4(mbilateral->Get_Cq_b()->GetElementN(0), mbilateral->Get_Cq_b()->GetElementN(1), mbilateral->Get_Cq_b()->GetElementN(2), 0);//J2x
			C = F4(mbilateral->Get_Cq_a()->GetElementN(3), mbilateral->Get_Cq_a()->GetElementN(4), mbilateral->Get_Cq_a()->GetElementN(5), 0);//J1w
			D = F4(mbilateral->Get_Cq_b()->GetElementN(3), mbilateral->Get_Cq_b()->GetElementN(4), mbilateral->Get_Cq_b()->GetElementN(5), 0);//J2w
			A.w = idA;//pointer to body B1 info in body buffer
			B.w = idB;//pointer to body B2 info in body buffer

			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 0] = A;
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 1] = B;
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 2] = C;
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 3] = D;
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 4].x = (1.0 / mbilateral->Get_g_i()); // eta = 1/g
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 4].y = mbilateral->Get_b_i(); // b_i is residual b
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 4].z = 0; //gammma, no warm starting
			gpu_solver->host_bilateral_data[counter + number_of_bilaterals * 4].w = (mbilateral->IsUnilateral()) ? 1 : 0;
			counter++;
		}// end loop
		mSystemDescriptor->number_of_bilaterals = number_of_bilaterals;
		// -3-  EXECUTE KERNELS ===============
		gpu_solver->c_factor = 1.0 / mDt;//gpu_contact_container->Get_load_C_factor();
		gpu_solver->tolerance = mTolerance;
		double maxrecspeed = gpu_contact_container->Get_load_max_recovery_speed();
		if (gpu_contact_container->Get_load_do_clamp() == false) {
			maxrecspeed = 10e25;
		}
		gpu_solver->negated_recovery_speed = -maxrecspeed;
		gpu_solver->step_size = mDt;
		gpu_solver->maximum_iterations = mMaxIterations;
		gpu_solver->force_factor = 1.0;
		gpu_solver->lcp_omega_bilateral = mOmegaBilateral;
		gpu_solver->lcp_omega_contact = mOmegaContact;
		gpu_solver->tolerance = mTolerance;
		gpu_solver->data_container = data_container;
		force_solver->data_container = data_container;
		//force_solver->ComputeForces();
		gpu_solver->number_of_bilaterals = number_of_bilaterals;
		gpu_solver->RunTimeStep();
		data_container->DeviceToHost();
#pragma omp parallel for
		for (unsigned int i = 0; i < mvariables.size(); i++) {

				float3 new_pos = data_container->host_pos_data[i];
				float4 new_rot = data_container->host_rot_data[i];
				float3 new_vel = data_container->host_vel_data[i];
				float3 new_acc = data_container->host_acc_data[i];
				float3 new_omg = data_container->host_omg_data[i];
				float3 new_fap = data_container->host_fap_data[i];

				ChLcpVariablesBody* mbodyvars = (ChLcpVariablesBody*) mvariables[i];
				ChBodyGPU* mbody = (ChBodyGPU*) mbodyvars->GetUserData();
				if (mbody->IsActive()) {
				mbody->SetPos(CHVECCAST(new_pos));
				mbody->SetRot(CHQUATCAST(new_rot));
				mbody->SetPos_dt(CHVECCAST(new_vel));
				mbody->SetPos_dtdt(CHVECCAST(new_acc));
				mbody->SetWvel_loc(CHVECCAST(new_omg));
				mbody->SetAppliedForce(CHVECCAST(new_fap));
			}
		}

		return 0;
	}

} // END_OF_NAMESPACE____
