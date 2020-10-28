#version 450 core
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;


layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoords;

layout (location = 0) out vec2 TexCoords;
layout (location = 1) out vec3 ViewPos;
layout (location = 2) out vec3 Normal;

void main()
{
    TexCoords = aTexCoords;
    ViewPos = vec3(pc.modelview * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(pc.modelview))) * aNormal;

    gl_Position =  pc.projection * pc.modelview * vec4(aPos, 1.0);
}
