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
struct __builtin_align__(16) object{
	float4 A,B,C;
	int2 family;
};

struct AABB{
	float4 min,max;
	int2 family;
};

namespace chrono {
	namespace collision {

		class ChApiGPU ChCCollisionGPU{
		public:
			ChCCollisionGPU();
			~ChCCollisionGPU();

			void Clear();
			void GetBounds();
			void TuneCD(int ss, int ee);
			void Run();
			void Broadphase();
			void Narrowphase();

			thrust::host_vector<object>		object_data_host;

			uint number_of_objects, 
				number_of_contacts, 
				last_active_bin,
				number_of_bin_intersections;

			float	bin_size,
				collision_envelope,
				optimal_bin_size,
				running_time,
				max_dimension;
			float3 
				max_bounding_point,
				min_bounding_point,
				global_origin;

			bool doTuning;
			cudaEvent_t start, stop;
			thrust::device_vector<contactGPU>* contact_data_gpu;
		private:
			thrust::device_vector<object>	object_data;
			thrust::device_vector<int3>		contact_pair;
			thrust::device_vector<uint>		generic_counter;
			thrust::device_vector<uint>		bin_number;
			thrust::device_vector<uint>		body_number;
			thrust::device_vector<AABB>		aabb_data;
			thrust::device_vector<uint>		bin_start_index;
		};
	}
}
#endif