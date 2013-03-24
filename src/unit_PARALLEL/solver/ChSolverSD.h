#ifndef CHSOLVERSD_H
#define CHSOLVERSD_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverSD: public ChSolverGPU {
    public:
        ChSolverSD();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveSD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);

};
}

#endif
