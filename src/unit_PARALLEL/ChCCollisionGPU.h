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
		ChCCollisionGPU();
		~ChCCollisionGPU() {
		}
		/// Do Proadphase Step
		static void Broadphase(
				device_vector<float3> &device_aabb_data,
				device_vector<int2> &device_fam_data,
				device_vector<int3> &device_typ_data,
				device_vector<long long> &contact_pair,
				uint &number_of_models,
				float3 &bin_size_vec,
				uint &number_of_contacts_possible);
		/// Do Narrowphase Step
		static void Narrowphase(
				device_vector<float3> &device_norm_data,
				device_vector<float3> &device_cpta_data,
				device_vector<float3> &device_cptb_data,
				device_vector<float> &device_dpth_data,
				device_vector<int2> &device_bids_data,
				device_vector<float3> &device_gam_data,
				device_vector<float3> &device_pos_data,
				device_vector<float4> &device_rot_data,
				device_vector<float3> &device_ObA_data,
				device_vector<float3> &device_ObB_data,
				device_vector<float3> &device_ObC_data,
				device_vector<float4> &device_ObR_data,
				device_vector<int3> &device_typ_data,
				device_vector<long long> &contact_pair,
				uint & number_of_contacts_possible,
				uint & number_of_contacts);
		/// Compute Axis Aligned Bounding Boxes for all collision geometries
		static void ComputeAABBHOST(ChGPUDataManager * data_container);
		static void ComputeAABB(
				uint &number_of_models,
				device_vector<float3> & device_pos_data,
				device_vector<float4> & device_rot_data,
				device_vector<float3> & device_ObA_data,
				device_vector<float3> & device_ObB_data,
				device_vector<float3> & device_ObC_data,
				device_vector<float4> & device_ObR_data,
				device_vector<int3> & device_typ_data,
				device_vector<float3> & device_aabb_data);
		/// Compute the bounds of the space
		static void ComputeBoundsHOST(ChGPUDataManager * data_container);
		static void ComputeBounds(
				uint & number_of_models,
				device_vector<float3> & device_aabb_data,
				float3& min_bounding_point,
				float3 &max_bounding_point);
		/// Update the location of the AABB
		static void UpdateAABBHOST(ChGPUDataManager * data_container);
		static void UpdateAABB(
				device_vector<float3> &device_aabb_data,
				float3 & min_bounding_point,
				float3 & max_bounding_point,
				float3 & bin_size_vec,
				float & max_dimension,
				float & collision_envelope,
				uint & number_of_models);


};
}
}
#endif
