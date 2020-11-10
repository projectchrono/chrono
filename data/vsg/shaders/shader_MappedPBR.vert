#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

layout(location = 0) out vec3 ViewPos;
layout(location = 1) out vec3 Normal;
layout(location = 2) out vec2 TexCoord;

void main()
{
    ViewPos = vec3(pc.modelview * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(pc.modelview))) * aNormal;
    TexCoord = aTexCoord;
    gl_Position =  pc.projection * pc.modelview * vec4(aPos, 1.0);
}
