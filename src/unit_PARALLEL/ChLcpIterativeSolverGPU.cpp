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
	mTolerance = 0;
	mDt = 0;
	mMaxIterations = 0;
	mOmegaContact = 0;
	mOmegaBilateral = 0;
}

ChLcpIterativeSolverGPUsimple::~ChLcpIterativeSolverGPUsimple() {
}


void ChLcpIterativeSolverGPUsimple::SolveSys(gpu_container & gpu_data) {
	gpu_solver->RunTimeStep(mMaxIterations, iteration_number, mDt, mOmegaBilateral, mOmegaContact, mTolerance, gpu_data);
}
void ChLcpIterativeSolverGPUsimple::SolveSys_HOST(ChGPUDataManager * data_container) {
	gpu_solver->RunTimeStep_HOST(mMaxIterations, iteration_number, mDt, mOmegaBilateral, mOmegaContact, mTolerance, data_container);
}
} // END_OF_NAMESPACE____
