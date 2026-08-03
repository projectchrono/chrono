#version 460
#extension GL_EXT_ray_tracing : require

struct HitPayload {
    vec4 hit_tuv;
    vec4 normal_object;
    vec4 tangent_flags;
    uvec4 ids_hit;
};

layout(location = 0) rayPayloadInEXT HitPayload payload;

void main() {
    payload.normal_object = vec4(0.0, 0.0, 0.0, 0.0);
    payload.tangent_flags = vec4(1.0, 0.0, 0.0, 0.0);
    payload.ids_hit = uvec4(0u, 0u, 0u, 0u);
}
