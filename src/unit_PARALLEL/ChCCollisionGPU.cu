#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"
__constant__ float3 global_origin_const;

using namespace chrono::collision;
// bounding box type
typedef thrust::pair<float3, float3> bbox;

// reduce a pair of bounding boxes (a,b) to a bounding box containing a and b
struct bbox_reduction: public thrust::binary_function<bbox, bbox, bbox> {
		__host__ __device__
		bbox operator()(bbox a, bbox b) {
			float3 ll = F3(fminf(a.first.x, b.first.x), fminf(a.first.y, b.first.y), fminf(
					a.first.z,
					b.first.z));// lower left corner
			float3 ur = F3(fmaxf(a.second.x, b.second.x), fmaxf(a.second.y, b.second.y), fmaxf(
					a.second.z,
					b.second.z));// upper right corner
			return bbox(ll, ur);
		}
};

// convert a point to a bbox containing that point, (point) -> (point, point)
struct bbox_transformation: public thrust::unary_function<float3, bbox> {
		__host__ __device__
		bbox operator()(float3 point) {
			return bbox(point, point);
		}
};

__global__ void Compute_AABBs(
		float3* pos,
		float4* rot,
		float3* obA,
		float3* obB,
		float3* obC,
		float4* obR,
		int3* typ,
		float3* aabb) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	int3 type = typ[index];

	float3 A = obA[index];
	float3 B = obB[index];
	float3 C = obC[index];

	float3 position = pos[type.z];
	float4 rotation = normalize(rot[type.z] + obR[index]);

	float3 temp_min;
	float3 temp_max;

	if (type.x == 0) {
		ComputeAABBSphere(B.x, A + position, temp_min, temp_max);
	} else if (type.x == 5) {
		A = quatRotate(A + position, rotation);
		B = quatRotate(B + position, rotation);
		C = quatRotate(C + position, rotation);

		ComputeAABBTriangle(A, B, C, temp_min, temp_max);
	} else if (type.x == 1 || type.x == 2 || type.x == 3) {
		ComputeAABBBox(B, A + position, rotation, temp_min, temp_max);
	} else {
		return;
	}
	aabb[index] = temp_min;
	aabb[index + number_of_models_const] = temp_max;
}

__global__ void Offset_AABBs(float3* points) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	float3 temp_min = points[index];
	float3 temp_max = points[index + number_of_models_const];
	points[index] = temp_min - F3(collision_envelope_const) + global_origin_const;
	points[index + number_of_models_const] = temp_max + F3(collision_envelope_const)
			+ global_origin_const;
}

ChCCollisionGPU::ChCCollisionGPU() {}
void ChCCollisionGPU::ComputeAABBHOST(ChGPUDataManager * data_container) {
//	data_container->host_aabb_data.resize(number_of_models * 2);
//	for (int index = 0; index < number_of_models; index++) {
//
//		int3 type = data_container->host_typ_data[index];
//
//		float3 A = data_container->host_ObA_data[index];
//		float3 B = data_container->host_ObB_data[index];
//		float3 C = data_container->host_ObC_data[index];
//
//		float3 position = data_container->host_pos_data[type.z];
//		float4 rotation = normalize(data_container->host_rot_data[type.z]
//				+ data_container->host_ObR_data[index]);
//
//		float3 temp_min;
//		float3 temp_max;
//
//		if (type.x == 0) {
//			ComputeAABBSphere(B.x, A + position, temp_min, temp_max);
//		} else if (type.x == 5) {
//			A = quatRotate(A + position, rotation);
//			B = quatRotate(B + position, rotation);
//			C = quatRotate(C + position, rotation);
//
//			ComputeAABBTriangle(A, B, C, temp_min, temp_max);
//		} else if (type.x == 1 || type.x == 2 || type.x == 3) {
//			ComputeAABBBox(B, A + position, rotation, temp_min, temp_max);
//		} else {
//			return;
//		}
//		data_container->host_aabb_data[index] = temp_min;
//		data_container->host_aabb_data[index + number_of_models] = temp_max;
//	}
}
void ChCCollisionGPU::ComputeBoundsHOST(ChGPUDataManager * data_container) {
//	bbox init = bbox(data_container->host_aabb_data[0], data_container->host_aabb_data[0]);
//	bbox_transformation unary_op;
//	bbox_reduction binary_op;
//	bbox result = thrust::transform_reduce(
//			data_container->host_aabb_data.begin(),
//			data_container->host_aabb_data.end(),
//			unary_op,
//			init,
//			binary_op);
//	data_container->min_bounding_point = result.first;
//	data_container->max_bounding_point = result.second;
}

void ChCCollisionGPU::UpdateAABBHOST(ChGPUDataManager * data_container) {
//	global_origin = fabs(data_container->min_bounding_point); //Determine Global Origin
//	max_dimension = max3(global_origin + fabs(data_container->max_bounding_point)); //Determine Max point in space
//	bin_size_vec = (global_origin + fabs(data_container->max_bounding_point)) / (powf(
//			number_of_models * 2,
//			1 / 3.0));
//	bin_size_vec = 1.0 / bin_size_vec;
//
//	for (int index = 0; index < number_of_models; index++) {
//		float3 temp_min = data_container->host_aabb_data[index];
//		float3 temp_max = data_container->host_aabb_data[index + number_of_models];
//		data_container->host_aabb_data[index] = temp_min - F3(collision_envelope) + global_origin;
//		data_container->host_aabb_data[index + number_of_models] = temp_max
//				+ F3(collision_envelope) + global_origin;
//
//		data_container->host_bin_data.resize(number_of_models * 2);
//
//		uint3 gmin = Hash(data_container->host_aabb_data[index] * 10.0);
//		uint3 gmax = Hash(data_container->host_aabb_data[index + number_of_models] * 10.0);
//		data_container->host_bin_data[index] = gmin;
//		data_container->host_bin_data[index + number_of_models] = gmax;
//	}
}

void ChCCollisionGPU::ComputeAABB(
		uint & number_of_models,
		device_vector<float3> & device_pos_data,
		device_vector<float4> & device_rot_data,
		device_vector<float3> & device_ObA_data,
		device_vector<float3> & device_ObB_data,
		device_vector<float3> & device_ObC_data,
		device_vector<float4> & device_ObR_data,
		device_vector<int3> & device_typ_data,
		device_vector<float3> & device_aabb_data) {
	COPY_TO_CONST_MEM(number_of_models);
	device_aabb_data.resize(number_of_models * 2);
	Compute_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(
			CASTF3(device_pos_data),
			CASTF4(device_rot_data),
			CASTF3(device_ObA_data),
			CASTF3(device_ObB_data),
			CASTF3(device_ObC_data),
			CASTF4(device_ObR_data),
			CASTI3(device_typ_data),
			CASTF3(device_aabb_data));
}
void ChCCollisionGPU::ComputeBounds(
		uint & number_of_models,
		device_vector<float3> & device_aabb_data,
		float3& min_bounding_point,
		float3 &max_bounding_point) {
	COPY_TO_CONST_MEM(number_of_models);
	bbox init = bbox(device_aabb_data[0], device_aabb_data[0]);
	bbox_transformation unary_op;
	bbox_reduction binary_op;
	bbox result = thrust::transform_reduce(
			device_aabb_data.begin(),
			device_aabb_data.end(),
			unary_op,
			init,
			binary_op);
	min_bounding_point = result.first;
	max_bounding_point = result.second;
}

__global__ void FindGrid(float3* AABBs, uint3* aabb_minmax) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	uint3 gmin = Hash(AABBs[index] * 10.0);
	uint3 gmax = Hash(AABBs[index + number_of_models_const] * 10.0);
	aabb_minmax[index] = gmin;
	aabb_minmax[index + number_of_models_const] = gmax;
}
__device__ __host__ bool operator ==(const uint3 &a, const uint3 &b) {
	return ((a.x == b.x) && (a.y == b.y) && (a.z == b.z));
}
void ChCCollisionGPU::UpdateAABB(
		device_vector<float3> &device_aabb_data,
		float3 & min_bounding_point,
		float3 & max_bounding_point,
		float3 & bin_size_vec,
		float & max_dimension,
		float & collision_envelope,
		uint & number_of_models) {
	float3 global_origin = fabs(min_bounding_point); //Determine Global Origin
	max_dimension = max3(global_origin + fabs(max_bounding_point)); //Determine Max point in space
	bin_size_vec = (global_origin + fabs(max_bounding_point))
			/ (powf(number_of_models * 2, 1 / 3.0));
	bin_size_vec = 1.0 / bin_size_vec;

	COPY_TO_CONST_MEM(number_of_models);
	COPY_TO_CONST_MEM(collision_envelope); //Contact Envelope
	COPY_TO_CONST_MEM(global_origin); //Origin for Physical Space
	COPY_TO_CONST_MEM(bin_size_vec);
	Offset_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(CASTF3(device_aabb_data));

	//	data_container->gpu_data[i].device_bin_data.resize(number_of_models * 2);
	//FindGrid<<<BLOCKS(number_of_models),THREADS>>>(CASTF3(data_container->gpu_data[i].device_aabb_data), CASTU3(data_container->gpu_data[i].device_bin_data));
}
