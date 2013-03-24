#ifndef CHSOLVERCG_H
#define CHSOLVERCG_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverCG: public ChSolverGPU {
    public:
        ChSolverCG();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
};
}

#endif
