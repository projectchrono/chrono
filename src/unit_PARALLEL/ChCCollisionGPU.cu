#include "ChCCollisionGPU.h"
#include "ChCCollisionGPU.cuh"
__constant__ real3 global_origin_const;
__constant__ uint number_of_models_const;
__constant__ real3 bin_size_vec_const;
__constant__ real collision_envelope_const;
using namespace chrono::collision;

__global__ void Compute_AABBs(real3 *pos, real4 *rot, real3 *obA, real3 *obB, real3 *obC, real4 *obR, int3 *typ, real3 *aabb) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index >= number_of_models_const) {
        return;
    }

    int3 type = typ[index];
    real3 A = obA[index];
    real3 B = obB[index];
    real3 C = obC[index];
    real3 position = pos[type.z];
    real4 rotation = (mult(rot[type.z], obR[index]));
    real3 temp_min;
    real3 temp_max;

    if (type.x == 0) {
        A = quatRotate(A, rot[type.z]);
        ComputeAABBSphere(B.x, A + position, temp_min, temp_max);
    } else if (type.x == 5) {
        A = quatRotate(A + position, rotation);
        B = quatRotate(B + position, rotation);
        C = quatRotate(C + position, rotation);
        ComputeAABBTriangle(A, B, C, temp_min, temp_max);
    } else if (type.x == 1 || type.x == 2 || type.x == 3) {
        //A=quatRotate(A,rot[type.z]);
        ComputeAABBBox(B, A , position, obR[index], rot[type.z] , temp_min, temp_max);
    } else {
        return;
    }

    aabb[index] = temp_min;
    aabb[index + number_of_models_const] = temp_max;
}

__global__ void Offset_AABBs(real3 *aabb) {
    uint index = blockIdx.x * blockDim.x + threadIdx.x;

    if (index >= number_of_models_const) {
        return;
    }

    real3 temp_min = aabb[index];
    real3 temp_max = aabb[index + number_of_models_const];
    aabb[index] = temp_min - R3(collision_envelope_const) + global_origin_const;
    aabb[index + number_of_models_const] = temp_max + R3(collision_envelope_const) + global_origin_const;
}


void ChCCollisionGPU::ComputeAABB(gpu_container &gpu_data) {
    uint number_of_models = gpu_data.number_of_models;
    COPY_TO_CONST_MEM(number_of_models);
    gpu_data.device_aabb_data.resize(number_of_models * 2);
    Compute_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models), THREADS)(CASTR3(gpu_data.device_pos_data), CASTR4(gpu_data.device_rot_data), CASTR3(gpu_data.device_ObA_data),
            CASTR3(gpu_data.device_ObB_data), CASTR3(gpu_data.device_ObC_data), CASTR4(gpu_data.device_ObR_data), CASTI3(gpu_data.device_typ_data), CASTR3(gpu_data.device_aabb_data));
    bbox init = bbox(gpu_data.device_aabb_data[0], gpu_data.device_aabb_data[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    bbox result = thrust::transform_reduce(gpu_data.device_aabb_data.begin(), gpu_data.device_aabb_data.end(), unary_op, init, binary_op);
    gpu_data.min_bounding_point = result.first;
    gpu_data.max_bounding_point = result.second;
    real3 global_origin = fabs(gpu_data.min_bounding_point);
    real collision_envelope = 0;
    COPY_TO_CONST_MEM(collision_envelope);
    COPY_TO_CONST_MEM(global_origin);
    Offset_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models), THREADS)(CASTR3(gpu_data.device_aabb_data));
}
void ChCCollisionGPU::ComputeBounds(gpu_container &gpu_data) {
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

void ChCCollisionGPU::UpdateAABB(real &collision_envelope, gpu_container &gpu_data, real3 global_origin) {
    uint number_of_models = gpu_data.number_of_models;
    COPY_TO_CONST_MEM(number_of_models);
    COPY_TO_CONST_MEM(collision_envelope);
    COPY_TO_CONST_MEM(global_origin);
    Offset_AABBs CUDA_KERNEL_DIM(BLOCKS(number_of_models), THREADS)(CASTR3(gpu_data.device_aabb_data));
    //  data_container->gpu_data[i].device_bin_data.resize(number_of_models * 2);
    //FindGrid<<<BLOCKS(number_of_models),THREADS>>>(CASTR3(data_container->gpu_data[i].device_aabb_data), CASTU3(data_container->gpu_data[i].device_bin_data));
}


