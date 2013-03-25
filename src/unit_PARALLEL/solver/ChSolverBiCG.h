#ifndef CHSOLVERBICG_H
#define CHSOLVERBICG_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverBiCG: public ChSolverGPU {
    public:
        ChSolverBiCG();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveBiCG(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
};
}

#endif
