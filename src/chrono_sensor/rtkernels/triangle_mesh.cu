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
// Authors: Asher Elmquist, Han Wang
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
    // triangle index and barycentrics
    const uint tri_id = rtGetPrimitiveIndex();
    const float2 bary_coord = rtGetTriangleBarycentrics();

    // vertices - get the vertex from the buffer - will exist for every mesh
    const uint3 vertex_idx = vertex_index_buffer[tri_id];
    const float3 v1 = vertex_buffer[vertex_idx.x];
    const float3 v2 = vertex_buffer[vertex_idx.y];
    const float3 v3 = vertex_buffer[vertex_idx.z];

    shading_normal = make_float3(0);
    tangent_vector = make_float3(0);
    texcoord = make_float2(0);

    // normals - normals may not exist so either pull normals from faces or buffer
    if (normal_index_buffer.size() <= tri_id || normal_buffer.size() <= normal_index_buffer[tri_id].x) {
        shading_normal = normalize(cross(v2 - v1, v3 - v1));
    } else {
        const uint3 n_id = normal_index_buffer[tri_id];
        shading_normal = normalize(normal_buffer[n_id.y] * bary_coord.x + normal_buffer[n_id.z] * bary_coord.y +
                                   normal_buffer[n_id.x] * (1.0f - bary_coord.x - bary_coord.y));
    }

    // texture coordinates - texcoords may not exist so either pull from buffer or set to 0
    if (texcoord_index_buffer.size() <= tri_id || texcoord_buffer.size() <= texcoord_index_buffer[tri_id].x) {
        texcoord = make_float2(0.0f);
    } else {
        const uint3 t_id = texcoord_index_buffer[tri_id];
        texcoord = texcoord_buffer[t_id.y] * bary_coord.x + texcoord_buffer[t_id.z] * bary_coord.y +
                   texcoord_buffer[t_id.x] * (1.0f - bary_coord.x - bary_coord.y);

        float3 e1 = v2 - v1;
        float3 e2 = v3 - v1;

        float2 delta_uv1 = texcoord_buffer[t_id.y] - texcoord_buffer[t_id.x];
        float2 delta_uv2 = texcoord_buffer[t_id.z] - texcoord_buffer[t_id.x];

        float f = 1.0f / (delta_uv1.x * delta_uv2.y - delta_uv2.x * delta_uv1.y);

        tangent_vector.x = f * (delta_uv2.y * e1.x - delta_uv1.y * e2.x);
        tangent_vector.y = f * (delta_uv2.y * e1.y - delta_uv1.y * e2.y);
        tangent_vector.z = f * (delta_uv2.y * e1.z - delta_uv1.y * e2.z);
        tangent_vector = normalize(tangent_vector);
    }
}
