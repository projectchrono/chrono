// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Bocheng Zou
// =============================================================================
//
// CUDA helpers for native Chrono::FSI::SPH rendering in Chrono::Sensor.
//
// =============================================================================

#include "chrono_sensor/cuda/fsi_sph_render.cuh"

namespace chrono {
namespace sensor {

namespace {

__device__ unsigned int hash_u32(unsigned int x) {
    x ^= x >> 16;
    x *= 0x7feb352dU;
    x ^= x >> 15;
    x *= 0x846ca68bU;
    x ^= x >> 16;
    return x;
}

__device__ float hash_unit(unsigned int seed) {
    return static_cast<float>(hash_u32(seed) & 0x00ffffffU) / static_cast<float>(0x01000000U);
}

__global__ void update_fsi_sph_sprite_instances(const chrono::fsi::sph::Real4* pos_rad,
                                                size_t count,
                                                OptixInstance* instances,
                                                const OptixTraversableHandle* gas_handles,
                                                const unsigned int* sbt_offsets,
                                                const float3* template_scales,
                                                unsigned int num_templates,
                                                float jitter_x,
                                                float jitter_y,
                                                float jitter_z,
                                                float origin_x,
                                                float origin_y,
                                                float origin_z) {
    const size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= count)
        return;

    const unsigned int seed = static_cast<unsigned int>(idx);
    const unsigned int template_id = hash_u32(seed) % num_templates;
    const float offset_x = (2.f * hash_unit(seed ^ 0x63d83595U) - 1.f) * jitter_x;
    const float offset_y = (2.f * hash_unit(seed ^ 0xb7e15162U) - 1.f) * jitter_y;
    const float offset_z = (2.f * hash_unit(seed ^ 0x8aed2a6bU) - 1.f) * jitter_z;

    const auto p = pos_rad[idx];
    const float x = static_cast<float>(p.x) + offset_x - origin_x;
    const float y = static_cast<float>(p.y) + offset_y - origin_y;
    const float z = static_cast<float>(p.z) + offset_z - origin_z;
    const float3 template_scale = template_scales[template_id];

    OptixInstance instance = {};
    instance.traversableHandle = gas_handles[template_id];
    instance.flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT;
    instance.instanceId = static_cast<unsigned int>(idx);
    instance.sbtOffset = sbt_offsets[template_id];
    instance.visibilityMask = 1;

    instance.transform[0] = template_scale.x;
    instance.transform[1] = 0.f;
    instance.transform[2] = 0.f;
    instance.transform[3] = x;
    instance.transform[4] = 0.f;
    instance.transform[5] = template_scale.y;
    instance.transform[6] = 0.f;
    instance.transform[7] = y;
    instance.transform[8] = 0.f;
    instance.transform[9] = 0.f;
    instance.transform[10] = template_scale.z;
    instance.transform[11] = z;

    instances[idx] = instance;
}

}  // namespace

void cuda_update_fsi_sph_sprite_instances(const chrono::fsi::sph::Real4* pos_rad,
                                          size_t count,
                                          OptixInstance* instances,
                                          const OptixTraversableHandle* gas_handles,
                                          const unsigned int* sbt_offsets,
                                          const float3* template_scales,
                                          unsigned int num_templates,
                                          const ChVector3f& position_jitter,
                                          const ChVector3f& origin_offset) {
    if (!pos_rad || !instances || !gas_handles || !sbt_offsets || !template_scales || count == 0 || num_templates == 0)
        return;

    constexpr int block_size = 256;
    const int grid_size = static_cast<int>((count + block_size - 1) / block_size);
    update_fsi_sph_sprite_instances<<<grid_size, block_size>>>(pos_rad, count, instances, gas_handles, sbt_offsets, template_scales, num_templates, position_jitter.x(),
                                                               position_jitter.y(), position_jitter.z(), origin_offset.x(), origin_offset.y(), origin_offset.z());
}

}  // namespace sensor
}  // namespace chrono
