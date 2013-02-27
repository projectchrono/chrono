#ifndef CHC_COLLISIONGPU_H
#define CHC_COLLISIONGPU_H
//////////////////////////////////////////////////
//
//   ChCCollisionGPU.h
//
//   GPU Collision Detection Algorithm
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//   Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCudaMath.h"
#include "ChCudaDefines.h"
#include "ChDataManager.h"

namespace chrono {
    namespace collision {

        class ChApiGPU ChCCollisionGPU {
            public:
                ChCCollisionGPU() {
                }
                ~ChCCollisionGPU() {
                }
                /// Do Broadphase Step
                static void Broadphase(gpu_container &gpu_data, bool tune);
                /// Do Narrowphase Step
                static void Narrowphase(gpu_container &gpu_data);
                /// Compute Axis Aligned Bounding Boxes for all collision geometries
                static void ComputeAABB(gpu_container &gpu_data);
                /// Compute the bounds of the space
                static void ComputeBounds(gpu_container &gpu_data);
                /// Update the location of the AABB
                static void UpdateAABB(real &collision_envelope, gpu_container &gpu_data, real3 global_origin);

        };
    }
}
#endif


