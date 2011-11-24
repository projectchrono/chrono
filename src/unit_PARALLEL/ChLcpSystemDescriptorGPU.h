#ifndef CHLCPSYSTEMDESCRIPTORGPU_H
#define CHLCPSYSTEMDESCRIPTORGPU_H
//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptorGPU.h
//
//   Contains GPU Memory shared between the Collision Detection and LCP Solver Algorithms
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "ChCuda.h"
#include <thrust/device_vector.h>
#include "ChCCollisionGPU.h"
#include "ChLcpIterativeSolverGPU.h"
namespace chrono {
	class ChApiGPU ChLcpSystemDescriptorGPU: public ChLcpSystemDescriptor {
		public:
			ChLcpSystemDescriptorGPU() {
				gpu_solver = new ChLcpIterativeSolverGPU();
				gpu_collision = new chrono::collision::ChCCollisionGPU();
			}

			~ChLcpSystemDescriptorGPU() {}

			ChLcpIterativeSolverGPU * gpu_solver;
			chrono::collision::ChCCollisionGPU * gpu_collision;

			uint number_of_contacts, number_of_bilaterals, number_of_bodies;
	};
} // END_OF_NAMESPACE____

#endif