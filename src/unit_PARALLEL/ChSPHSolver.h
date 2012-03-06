#ifndef CH_SPHGPU_H
#define CH_SPHGPU_H

//////////////////////////////////////////////////
//
//   CHSPHSolver.h
//
//   GPU Data Manager Class
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCuda.h"
#include "ChDataManager.h"
#include <vector>

using namespace std;

namespace chrono {

	class ChApiGPU CHSPHSystem {
		public:
			CHSPHSystem(){}
			//void Simulate(gpu_container & gpu_data);
	};
}

#endif
