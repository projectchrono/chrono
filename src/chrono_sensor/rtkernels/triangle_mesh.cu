// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2019 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// RT kernels for mesh geometries
//
// =============================================================================

#include <optixu/optixu_math_namespace.h>

using namespace optix;

rtDeclareVariable(float3, shading_normal, attribute shading_normal, );
rtDeclareVariable(float3, tangent_vector, attribute tangent_vector, );
rtDeclareVariable(float2, texcoord, attribute texcoord, );

rtBuffer<uint3> vertex_index_buffer;
rtBuffer<uint3> normal_index_buffer;
rtBuffer<uint3> texcoord_index_buffer;
rtBuffer<float3> vertex_buffer;
rtBuffer<float3> normal_buffer;
rtBuffer<float2> texcoord_buffer;

RT_PROGRAM void mesh_attributes() {
    const uint3 vertex_idx = vertex_index_buffer[rtGetPrimitiveIndex()];

    uint3 texcoord_idx;
    uint3 normal_idx;

    if (normal_buffer.size() > 0 && normal_index_buffer.size() > 0) {
        normal_idx = normal_index_buffer[rtGetPrimitiveIndex()];
    } else if (normal_buffer.size() > 0) {
        normal_idx = vertex_idx;  // vertex_index_buffer[rtGetPrimitiveIndex()];
    }

    if (texcoord_buffer.size() > 0 && texcoord_index_buffer.size() > 0) {
        texcoord_idx = texcoord_index_buffer[rtGetPrimitiveIndex()];
    } else if (texcoord_buffer.size() > 0) {
        texcoord_idx = vertex_idx;  // vertex_index_buffer[rtGetPrimitiveIndex()];
    }

    const float2 bary_coord = rtGetTriangleBarycentrics();

    if (normal_buffer.size() > 0) {
        shading_normal =
            normalize(normal_buffer[normal_idx.y] * bary_coord.x + normal_buffer[normal_idx.z] * bary_coord.y +
                      normal_buffer[normal_idx.x] * (1.0f - bary_coord.x - bary_coord.y));
    } else {
        shading_normal = normalize(cross(vertex_buffer[vertex_idx.y] - vertex_buffer[vertex_idx.x],
                                         vertex_buffer[vertex_idx.z] - vertex_buffer[vertex_idx.x]));
    }
    if (texcoord_buffer.size() == 0) {
        texcoord = make_float2(0.0f);
    } else {
        texcoord = texcoord_buffer[texcoord_idx.y] * bary_coord.x + texcoord_buffer[texcoord_idx.z] * bary_coord.y +
                   texcoord_buffer[texcoord_idx.x] * (1.0f - bary_coord.x - bary_coord.y);
    }

    // calculating tangent vector
    float3 e1 = vertex_buffer[vertex_idx.y] - vertex_buffer[vertex_idx.x];
    float3 e2 = vertex_buffer[vertex_idx.z] - vertex_buffer[vertex_idx.x];
    //    printf("%.3f %.3f %.3f\n", vertex_buffer[vertex_idx.x].x, vertex_buffer[vertex_idx.x].y,
    //    vertex_buffer[vertex_idx.x].z);

    float2 delta_uv1 = texcoord_buffer[texcoord_idx.y] - texcoord_buffer[texcoord_idx.x];
    float2 delta_uv2 = texcoord_buffer[texcoord_idx.z] - texcoord_buffer[texcoord_idx.x];

    //    printf("%.3f %.3f\n", delta_uv2.x, delta_uv2.y);
    float f = 1.0f / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);

    tangent_vector.x = f * (delta_uv2.y * e1.x - delta_uv1.y * e2.x);
    tangent_vector.y = f * (delta_uv2.y * e1.y - delta_uv1.y * e2.y);
    tangent_vector.z = f * (delta_uv2.y * e1.z - delta_uv1.y * e2.z);
    //    printf("%.3f %.3f %.3f    %.3f %.3f %.3f    %.3f %.3f   %.3f  %.3f\n", e1.x, e1.y, e1.z, e2.x, e2.y, e2.z,
    //    delta_uv1.x, delta_uv1.y, delta_uv2.x, delta_uv2.y);

    normalize(tangent_vector);
}
