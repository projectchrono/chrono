#ifndef CHSOLVERBICGSTAB_H
#define CHSOLVERBICGSTAB_H

#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverBiCGStab: public ChSolverGPU {
    public:
        ChSolverBiCGStab();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveBiCGStab(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);
};
}

#endif
