#version 330

layout(location = 0) in vec3 vertex_position;
layout(location = 2) in vec3 vertex_color;
layout(location = 1) in vec3 vertex_normal;


uniform mat4 projection_matrix;
uniform mat4 view_matrix;

flat out vec3 color;

void main() {
  mat4 modelview = view_matrix;
  mat4 mvp = projection_matrix * modelview;
  gl_Position = mvp * vec4(vertex_position, 1.0);
  color = vertex_color;
}
