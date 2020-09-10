#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;

layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_color;

layout(location = 0) out vec3 color;

void main() {
  mat4 mvp = pc.projection * pc.modelview;
  gl_Position = mvp * vec4(vertex_position, 1.0);
  color = vertex_color;
}
