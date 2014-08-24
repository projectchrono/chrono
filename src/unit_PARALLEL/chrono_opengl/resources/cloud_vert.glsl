#version 330

layout (location = 0) in vec3 vertex_position;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;

void main()
{
mat4 mvp = projection_matrix*view_matrix;
gl_Position = mvp * vec4(vertex_position, 1.0);
}
