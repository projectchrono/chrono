#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoords;
layout (location = 2) in vec3 aNormal;


layout(binding=0) uniform TF {
    mat4 model;
} tf;

layout(location=0) out vec2 TexCoords;
layout(location=1) out vec3 WorldPos;
layout(location=2) out vec3 Normal;
layout(location=3) out vec3 camPos;

void main()
{
    TexCoords = aTexCoords;
    WorldPos = vec3(tf.model * vec4(aPos, 1.0));
    Normal = mat3(tf.model) * aNormal;   
    camPos = vec3(pc.modelview * vec4(aPos, 1.0));
    gl_Position =  pc.projection * pc.modelview * vec4(aPos, 1.0);
}