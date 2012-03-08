#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"
__constant__ float3 global_origin_const;
__constant__ uint number_of_models_const;
__constant__ float3 bin_size_vec_const;
__constant__ float collision_envelope_const;
using namespace chrono::collision;

__global__ void Compute_AABBs(float3* pos, float4* rot, float3* obA, float3* obB, float3* obC, float4* obR, int3* typ, float3* aabb) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	int3 type = typ[index];

	float3 A = obA[index];
	float3 B = obB[index];
	float3 C = obC[index];

	float3 position = pos[type.z];
	float4 rotation = (mult(rot[type.z], obR[index]));

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

__global__ void Offset_AABBs(float3* aabb) {
	uint index = blockIdx.x * blockDim.x + threadIdx.x;
	if (index >= number_of_models_const) {
		return;
	}
	float3 temp_min = aabb[index];
	float3 temp_max = aabb[index + number_of_models_const];
	aabb[index] = temp_min - F3(collision_envelope_const) + global_origin_const;
	aabb[index + number_of_models_const] = temp_max + F3(collision_envelope_const) + global_origin_const;
}

void ChCCollisionGPU::ComputeAABB_HOST(ChGPUDataManager * data_container) {
	uint number_of_models = data_container->number_of_models;
	data_container->host_aabb_data.resize(number_of_models * 2);

	for (int index = 0; index < number_of_models; index++) {
		int3 type = data_container->host_typ_data[index];

		float3 A = data_container->host_ObA_data[index];
		float3 B = data_container->host_ObB_data[index];
		float3 C = data_container->host_ObC_data[index];

		float3 position = data_container->host_pos_data[type.z];
		float4 rotation = normalize(data_container->host_rot_data[type.z] + data_container->host_ObR_data[index]);

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
		data_container->host_aabb_data[index] = temp_min;
		data_container->host_aabb_data[index + number_of_models] = temp_max;
	}
}
void ChCCollisionGPU::ComputeBounds_HOST(ChGPUDataManager * data_container) {
	uint number_of_models = data_container->number_of_models;
	bbox init = bbox(data_container->host_aabb_data[0], data_container->host_aabb_data[0]);
	bbox_transformation unary_op;
	bbox_reduction binary_op;
	bbox result = thrust::transform_reduce(data_container->host_aabb_data.begin(), data_container->host_aabb_data.end(), unary_op, init, binary_op);
	data_container->min_bounding_point = result.first;
	data_container->max_bounding_point = result.second;
}
void ChCCollisionGPU::UpdateAABB_HOST(float3 & bin_size_vec, float & max_dimension, float & collision_envelope, ChGPUDataManager * data_container) {
	uint number_of_models = data_container->number_of_models;
	float3 global_origin = fabs(data_container->min_bounding_point); //Determine Global Origin
	max_dimension = max3(global_origin + fabs(data_container->max_bounding_point)); //Determine Max point in space
	bin_size_vec = (global_origin + fabs(data_container->max_bounding_point)) / (powf(number_of_models * 2, 1 / 3.0));
	bin_size_vec = 1.0 / bin_size_vec;

	for (int index = 0; index < number_of_models; index++) {
		float3 temp_min = data_container->host_aabb_data[index];
		float3 temp_max = data_container->host_aabb_data[index + number_of_models];
		data_container->host_aabb_data[index] = temp_min - F3(collision_envelope) + global_origin;
		data_container->host_aabb_data[index + number_of_models] = temp_max + F3(collision_envelope) + global_origin;
	}
}

void ChCCollisionGPU::ComputeAABB(gpu_container & gpu_data) {
	uint number_of_models = gpu_data.number_of_models;
	COPY_TO_CONST_MEM(number_of_models);
	gpu_data.device_aabb_data.resize(number_of_models * 2);
	Compute_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(CASTF3(gpu_data.device_pos_data), CASTF4(gpu_data.device_rot_data), CASTF3(gpu_data.device_ObA_data),
			CASTF3(gpu_data.device_ObB_data), CASTF3(gpu_data.device_ObC_data), CASTF4(gpu_data.device_ObR_data), CASTI3(gpu_data.device_typ_data), CASTF3(gpu_data.device_aabb_data));

	bbox init = bbox(gpu_data.device_aabb_data[0], gpu_data.device_aabb_data[0]);
	bbox_transformation unary_op;
	bbox_reduction binary_op;
	bbox result = thrust::transform_reduce(gpu_data.device_aabb_data.begin(), gpu_data.device_aabb_data.end(), unary_op, init, binary_op);
	gpu_data.min_bounding_point = result.first;
	gpu_data.max_bounding_point = result.second;

	float3 global_origin=fabs(gpu_data.min_bounding_point);
	float collision_envelope=0;
	COPY_TO_CONST_MEM(collision_envelope);
	COPY_TO_CONST_MEM(global_origin);
	Offset_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(CASTF3(gpu_data.device_aabb_data));
}
void ChCCollisionGPU::ComputeBounds(gpu_container & gpu_data) {
	uint number_of_models = gpu_data.number_of_models;
	COPY_TO_CONST_MEM(number_of_models);
	bbox init = bbox(gpu_data.device_aabb_data[0], gpu_data.device_aabb_data[0]);
	bbox_transformation unary_op;
	bbox_reduction binary_op;
	bbox result = thrust::transform_reduce(gpu_data.device_aabb_data.begin(), gpu_data.device_aabb_data.end(), unary_op, init, binary_op);
	gpu_data.min_bounding_point = result.first;
	gpu_data.max_bounding_point = result.second;
	//DBG("ComputeBounds");
}

void ChCCollisionGPU::UpdateAABB(float & collision_envelope, gpu_container & gpu_data, float3 global_origin) {
	uint number_of_models = gpu_data.number_of_models;
	COPY_TO_CONST_MEM(number_of_models);
	COPY_TO_CONST_MEM(collision_envelope);
	COPY_TO_CONST_MEM(global_origin);
	Offset_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models),THREADS)(CASTF3(gpu_data.device_aabb_data));

	//	data_container->gpu_data[i].device_bin_data.resize(number_of_models * 2);
	//FindGrid<<<BLOCKS(number_of_models),THREADS>>>(CASTF3(data_container->gpu_data[i].device_aabb_data), CASTU3(data_container->gpu_data[i].device_bin_data));
}
