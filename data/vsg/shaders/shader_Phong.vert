#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColorAmbient;
layout(location = 3) in vec3 inColorDiffuse;

layout(location = 0) out vec3 ambientColor;
layout(location = 1) out vec3 diffuseColor;
layout(location = 2) out vec3 specularColor;
layout(location = 3) out vec3 normal;
layout(location = 4) out vec3 eye_vec;

out gl_PerVertex {
    vec4 gl_Position;
};

void main() {
    gl_Position = (pc.projection * pc.modelview) * vec4(inPosition, 1.0);
    
    normal = normalize(mat3(pc.modelview) * inNormal);
    eye_vec = vec3(pc.modelview * vec4(inPosition, 1.0));

    ambientColor = inColorAmbient;
    diffuseColor = inColorDiffuse;
    specularColor = vec3(1, 1, 1);

}
