#ifndef CHSOLVERCG_H
#define CHSOLVERCG_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChOptimizationJacobi.h"
#include "ChSolverGPU.h"
#include "ChDataManager.h"
namespace chrono {
	class ChApiGPU ChSolverCG: public ChSolverGPU {
		public:
			ChSolverCG();
			void Solve(real step, gpu_container &gpu_data);
			uint SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
			uint SolveSD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);


		};}

#endif
