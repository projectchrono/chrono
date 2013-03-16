#ifndef CHSOLVERJACOBI_H
#define CHSOLVERJACOBI_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChThrustLinearAlgebra.cuh"
#include "ChOptimizationJacobi.h"
#include "ChSolverGPU.h"

namespace chrono {
    class ChApiGPU ChSolverJacobi: public ChSolverGPU {
        public:
    	ChSolverJacobi();

            void Solve();
        private:

            int current_iteration;
            int max_iteration;
            real residual;
            real tolerance;
            real epsilon;



    };

}




#endif
