#ifndef CHSOLVERGD_H
#define CHSOLVERGD_H
#include "ChSolverGPU.h"

namespace chrono {
class ChApiGPU ChSolverGD: public ChSolverGPU {
    public:
        ChSolverGD();
        void Solve(real step, gpu_container &gpu_data);
        uint SolveGD(custom_vector<real> &x, const custom_vector<real> &b, const uint max_iter);

};
}

#endif
