#version 400


layout (location = 0) in vec3 vertex_position;
layout (location = 1) in vec3 vertex_normal;
layout (location = 2) in vec3 vertex_color_ambient;
layout (location = 3) in vec3 vertex_color_diffuse;
layout (location = 4) in vec3 vertex_color_specular;

uniform mat4 mvp;
uniform mat4 modelview_matrix;
uniform mat3 normal_matrix;

vec3 light_pos = vec3(100,100,100);

flat out vec3 color_ambient;
flat out vec3 color_diffuse;
flat out vec3 color_specular;
out vec3 normal;
out vec3 light_dir;
out vec3 eye_vec;
void main()
{
    gl_Position = mvp * vec4(vertex_position, 1.0);

    color_ambient = vertex_color_ambient;
    color_diffuse = vertex_color_diffuse;
    color_specular = vertex_color_specular;
    normal= vertex_normal;
    light_dir = light_pos - vertex_position ;
    eye_vec = -vertex_position;
}
