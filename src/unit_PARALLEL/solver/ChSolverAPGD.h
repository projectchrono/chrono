#ifndef CHSOLVERAPGD_H
#define CHSOLVERAPGD_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverAPGD: public ChSolverGPU {
    public:
	ChSolverAPGD();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveAPGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
};
}

#endif
