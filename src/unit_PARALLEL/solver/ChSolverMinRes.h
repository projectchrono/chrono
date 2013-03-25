#ifndef CHSOLVERMINRES_H
#define CHSOLVERMINRES_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverMinRes: public ChSolverGPU {
    public:
	ChSolverMinRes();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveMinRes(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
};
}

#endif
