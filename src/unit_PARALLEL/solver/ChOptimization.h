#ifndef CHOPTIMIZATION_H
#define CHOPTIMIZATION_H

#include "ChCudaMath.h"
#include "ChCudaDefines.h"

namespace chrono {
    class ChApiGPU ChOptimization {
        public:
    	ChOptimization(){}
    	virtual custom_vector<real> LinearOperator(){}
    		virtual void Project(){}
    		virtual void Rhs(){}
    	    custom_vector<real> Ax,b,x;
    };

}

#endif
