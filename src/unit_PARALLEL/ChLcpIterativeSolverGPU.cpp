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
	ChLcpIterativeSolverGPUsimple::ChLcpIterativeSolverGPUsimple(ChContactContainerGPUsimple* container) {
		gpu_contact_container = container;
		gpu_solver = new ChLcpIterativeGPU();
		number_of_bodies = 0;
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
		// -3-  EXECUTE KERNELS ===============
		gpu_solver->c_factor = 1.0 / mDt;
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
		gpu_solver->RunTimeStep(0);

		return 0;
	}
} // END_OF_NAMESPACE____
