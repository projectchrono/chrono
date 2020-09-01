#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(push_constant) uniform PushConstants {
    mat4 projection;
    mat4 modelview;
} pc;


layout(location = 0) in vec3 vertex_position;
layout(location = 1) in vec3 vertex_normal;
layout(location = 2) in vec3 vertex_color_ambient;
layout(location = 3) in vec3 vertex_color_diffuse;
layout(location = 4) in vec3 vertex_color_specular;
layout(location = 5) in float vertex_shininess;
layout(location = 6) in float vertex_opacity;

layout(location = 0) out vec3 color_ambient;
layout(location = 1) out vec3 color_diffuse;
layout(location = 2) out vec3 color_specular;
layout(location = 3) out float shininess;
layout(location = 4) out float opacity;
layout(location = 5) out vec3 normal;
layout(location = 6) out vec3 eye_vec;

void main() {
  mat4 modelview = pc.modelview;
  mat4 mvp = pc.projection * modelview;
  // mat3 normalmatrix = inverse(transpose(mat3(modelview)));;

  gl_Position = mvp * vec4(vertex_position, 1.0);
  normal = normalize(mat3(modelview) * vertex_normal);
  eye_vec = vec3(modelview * vec4(vertex_position, 1.0));

  color_ambient = vertex_color_ambient;
  color_diffuse = vertex_color_diffuse;
  color_specular = vertex_color_specular;
  shininess = vertex_shininess;
  opacity = vertex_opacity;
}
