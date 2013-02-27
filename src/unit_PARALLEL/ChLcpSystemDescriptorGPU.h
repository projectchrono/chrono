#ifndef CHLCPSYSTEMDESCRIPTORGPU_H
#define CHLCPSYSTEMDESCRIPTORGPU_H
//////////////////////////////////////////////////
//
//   ChLcpSystemDescriptorGPU.h
//
//   Contains GPU Memory shared between the Collision Detection and LCP Solver Algorithms
//
//
//   HEADER file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChLcpSolverGPU.h"
namespace chrono {
    class ChApiGPU ChLcpSystemDescriptorGPU: public ChLcpSystemDescriptor {
        public:
            ChLcpSystemDescriptorGPU() {}
            ~ChLcpSystemDescriptorGPU() {}
    };
} // END_OF_NAMESPACE____

#endif


