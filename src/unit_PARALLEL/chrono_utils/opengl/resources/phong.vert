#version 330

layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec3 vertex_color_ambient;
layout (location = 3) in vec3 vertex_color_diffuse;
layout (location = 4) in vec3 vertex_color_specular;

uniform mat4 mvp;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;

flat out vec3 color_ambient;
flat out vec3 color_diffuse;
flat out vec3 color_specular;
out vec3 normal;
out vec3 eye_vec;
void main()
{
gl_Position = mvp * vec4(vertex_position, 1.0);

normal = normalize(normal_matrix* vertex_normal);
eye_vec = vec3(modelview_matrix* vec4(vertex_position, 1.0));

color_ambient = vertex_color_ambient;
color_diffuse = vertex_color_diffuse;
color_specular = vertex_color_specular;
}
