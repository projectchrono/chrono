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
#include "ChGPUDataManager.h"

struct AABB {
		float3 min, max;
};

namespace chrono {
	namespace collision {

		class ChApiGPU ChCCollisionGPU {
			public:
				ChCCollisionGPU();
				~ChCCollisionGPU();

				void Clear();
				void SyncData();
				void TuneCD(int ss, int ee);
				void Run();
				void Broadphase();
				void Narrowphase();
				void ComputeAABB();
				void ComputeBounds();
				void UpdateAABB();
				void AddObject(const float3 &A, const float3 &B, const float3 &C, const float4 &R, const int2 &F, const int3 &T);

				uint number_of_models,number_of_contacts, number_of_contacts_possible, last_active_bin, number_of_bin_intersections;

				float collision_envelope, optimal_bin_size, running_time, max_dimension;
				float3 max_bounding_point, min_bounding_point, global_origin, bin_size_vec;
				bool do_compute_bounds;
				cudaEvent_t start, stop;

				ChGPUDataManager * data_container;

				float3 init_min;
				float3 init_max;
			private:
				thrust::device_vector<long long> contact_pair;
				thrust::device_vector<long long> old_contact_pair;
				thrust::device_vector<uint> generic_counter;
				thrust::device_vector<uint> bin_number;
				thrust::device_vector<uint> body_number;
				thrust::device_vector<uint3> aabb_min_max_bin;
				thrust::device_vector<uint3> aabb_min_max_bin_old;
				thrust::device_vector<float3> aabb_data;
				thrust::device_vector<uint> bin_start_index;
		};
	}
}
#endif
