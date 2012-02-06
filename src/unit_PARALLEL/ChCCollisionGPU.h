#ifndef CHC_COLLISIONGPU_H
#define CHC_COLLISIONGPU_H
//////////////////////////////////////////////////
//
//   ChCCollisionGPU.h
//
//   GPU Collision Detection Algorithm
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChCuda.h"
#include "ChDataManager.h"

struct AABB {
		float3 min, max;
};

namespace chrono {
	namespace collision {

		class ChApiGPU ChCCollisionGPU {
			public:
				ChCCollisionGPU() {
				}
				~ChCCollisionGPU() {
				}
				/// Do Broadphase Step
				static void Broadphase(float3 &bin_size_vec, gpu_container & gpu_data);
				/// Do Narrowphase Step
				static void Narrowphase(gpu_container & gpu_data);
				/// Compute Axis Aligned Bounding Boxes for all collision geometries
				static void ComputeAABB(gpu_container & gpu_data);
				/// Compute the bounds of the space
				static void ComputeBounds(gpu_container & gpu_data);
				/// Update the location of the AABB
				static void UpdateAABB(float3 & bin_size_vec, float & max_dimension, float & collision_envelope, gpu_container & gpu_data);

				static void ComputeAABB_HOST(ChGPUDataManager * data_container);
				static void ComputeBounds_HOST(ChGPUDataManager * data_container);
				static void UpdateAABB_HOST(float3 & bin_size_vec, float & max_dimension, float & collision_envelope, ChGPUDataManager * data_container);
				static void Broadphase_HOST(float3 &bin_size_vec, ChGPUDataManager * data_container);
				static void Narrowphase_HOST(ChGPUDataManager * data_container);

		};
	}
}
#endif
