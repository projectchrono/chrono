#version 330

layout (location = 0) in vec3 vertex_position;

uniform mat4 mvp;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;


void main()
{
gl_Position = mvp * vec4(vertex_position, 1.0);
}
