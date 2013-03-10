#ifndef CHSOLVERGPU_H
#define CHSOLVERGPU_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChOptimization.h"

namespace chrono {
    class ChApiGPU ChSolverGPU {
        public:
            ChSolverGPU(){}

            void Solve(ChOptimization &optim){}
        private:

            int current_iteration;
            int max_iteration;
            real residual;
            real tolerance;
            real epsilon;

    };

}




#endif
