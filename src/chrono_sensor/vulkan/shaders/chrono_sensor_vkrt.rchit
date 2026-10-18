#version 460
#extension GL_EXT_ray_tracing : require

struct GpuMaterial {
    vec4 diffuse;
    vec4 specular;
    vec4 emissive;
    vec4 params;
    uvec4 ids;
    vec4 sensor;
    uvec4 texture0;
    uvec4 texture1;
    vec4 tex_scale;
};

struct GpuVertex {
    vec4 pos;
    vec4 normal;
    vec4 uv;
    vec4 tangent;
};

struct GpuTriangle {
    uvec4 index_material;
};

struct HitPayload {
    vec4 hit_tuv;        // hit distance, interpolated uv, reserved
    vec4 normal_object;  // normal xyz, object id
    vec4 tangent_flags;  // tangent xyz, has_uv
    uvec4 ids_hit;       // class id, instance id, hit flag, material index
};

layout(std430, set = 0, binding = 1) readonly buffer Materials {
    GpuMaterial materials[];
};
layout(std430, set = 0, binding = 2) readonly buffer Vertices {
    GpuVertex vertices[];
};
layout(std430, set = 0, binding = 3) readonly buffer Triangles {
    GpuTriangle triangles[];
};

hitAttributeEXT vec2 attribs;
layout(location = 0) rayPayloadInEXT HitPayload payload;

void main() {
    GpuTriangle tri = triangles[gl_PrimitiveID];
    uint i0 = tri.index_material.x;
    uint i1 = tri.index_material.y;
    uint i2 = tri.index_material.z;
    uint mat_id = tri.index_material.w;

    float b1 = attribs.x;
    float b2 = attribs.y;
    float b0 = 1.0 - b1 - b2;

    vec3 n = normalize(vertices[i0].normal.xyz * b0 + vertices[i1].normal.xyz * b1 + vertices[i2].normal.xyz * b2);
    vec3 t = normalize(vertices[i0].tangent.xyz * b0 + vertices[i1].tangent.xyz * b1 + vertices[i2].tangent.xyz * b2);
    vec2 uv = vertices[i0].uv.xy * b0 + vertices[i1].uv.xy * b1 + vertices[i2].uv.xy * b2;
    float has_uv = max(vertices[i0].uv.z, max(vertices[i1].uv.z, vertices[i2].uv.z));

    // Do not face-forward the staged shading normal based on triangle winding.
    // Chrono/OptiX keeps object/mesh normals in material space and uses them
    // directly for light sampling. The generated Vulkan primitive triangles
    // intentionally stage explicit normals, and several generated faces have a
    // winding opposite to that staged normal. Flipping here makes lit surfaces
    // point away from Point/Spot/Directional lights, producing visible material
    // colors from ambient/background only but no direct illumination or shadows.

    GpuMaterial mat = materials[mat_id];
    payload.hit_tuv = vec4(gl_HitTEXT, uv, 0.0);
    payload.normal_object = vec4(n, mat.sensor.z);
    payload.tangent_flags = vec4(t, has_uv);
    payload.ids_hit = uvec4(mat.ids.x, mat.ids.y, 1u, mat_id);
}
