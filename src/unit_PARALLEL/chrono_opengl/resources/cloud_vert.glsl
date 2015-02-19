#version 330

layout(location = 0) in vec3 vertex_position;

uniform mat4 projection_matrix;
uniform mat4 view_matrix;
uniform ivec2 viewport;
uniform float point_size;
void main() {
  float fovy = 45;  // degrees
  float heightOfNearPlane = float(abs(viewport.y)) / (2.0 * tan(0.5 * fovy * 3.1415 / 180.0));
  mat4 mvp = projection_matrix * view_matrix;
  gl_Position = mvp * vec4(vertex_position, 1.0);
  gl_PointSize = (heightOfNearPlane * point_size) / gl_Position.w;
}
