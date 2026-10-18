#version 460
#extension GL_EXT_ray_tracing : require

struct ShadowPayload {
    vec4 attenuation_remaining;
};

layout(location = 1) rayPayloadInEXT ShadowPayload shadow_payload;

void main() {
    // Opaque fast-shadow rays initialize attenuation to zero.  Reaching miss
    // means the light is visible, so set transmittance to one.  Transparent
    // scenes still use the material-aware closest-hit loop in raygen.
    shadow_payload.attenuation_remaining.rgb = vec3(1.0);
}
