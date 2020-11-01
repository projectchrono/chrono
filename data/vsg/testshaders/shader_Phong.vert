#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 ambientColor;
layout (location = 3) in vec3 diffuseColor;
layout (location = 4) in vec3 specularColor;
layout (location = 5) in float shininess;

layout(location = 0) out vec3 FragPos;
layout(location = 1) out vec3 Normal;
layout(location = 2) out vec3 AmbientColor;
layout(location = 3) out vec3 DiffuseColor;
layout(location = 4) out vec3 SpecularColor;
layout(location = 5) out float Shininess;

void main()
{
    gl_Position = pc.projection * pc.modelview * vec4(aPos, 1.0);
    FragPos = vec3(pc.modelview * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(pc.modelview))) * aNormal;
    //LightPos = vec3(view * vec4(lightPos, 1.0)); // Transform world-space light position to 
    AmbientColor = ambientColor;
    DiffuseColor = diffuseColor;
    SpecularColor = specularColor;
    Shininess = shininess;
}

