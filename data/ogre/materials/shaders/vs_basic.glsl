#version 150

uniform vec4 
uniform mat4 worldViewProjMatrix;

in vec4 vertex;
in vec3 normal;

void main(void)
{
    vec3 Pos = vertex.xyz;

    gl_Position = worldViewProjMatrix * vec4(Pos,1.0);
}
