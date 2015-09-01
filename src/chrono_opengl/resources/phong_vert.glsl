#version 330

layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_normal;
layout(location = 2) in vec3 vertex_color_ambient;
layout(location = 3) in vec3 vertex_color_diffuse;
layout(location = 4) in mat4 model_matrix;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;

flat out vec3 color_ambient;
flat out vec3 color_diffuse;
flat out vec3 color_specular;
out vec3 normal;
out vec3 eye_vec;
void main() {
  mat4 modelview = view_matrix * model_matrix;
  mat4 mvp = projection_matrix * modelview;
  // mat3 normalmatrix = inverse(transpose(mat3(modelview)));;

  gl_Position = mvp * vec4(vertex_position, 1.0);
  normal = normalize(mat3(modelview) * vertex_normal);
  eye_vec = vec3(modelview * vec4(vertex_position, 1.0));

  color_ambient = vertex_color_ambient;
  color_diffuse = vertex_color_diffuse;
}
