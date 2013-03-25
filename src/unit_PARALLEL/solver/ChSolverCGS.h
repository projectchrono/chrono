#ifndef CHSOLVERCGS_H
#define CHSOLVERCGS_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverCGS: public ChSolverGPU {
    public:
        ChSolverCGS();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveCGS(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
};
}

#endif
