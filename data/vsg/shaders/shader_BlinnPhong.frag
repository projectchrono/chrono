#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(set = 1, binding = 0) uniform LightSettings {
    vec3 light_pos;
} lightSettings;

layout(location = 0) out vec4 FragColor;

layout(location = 0) in vec3 color_ambient;
layout(location = 1) in vec3 color_diffuse;
layout(location = 2) in vec3 color_specular;
layout(location = 3) in float shininess;
layout(location = 4) in float opacity;
layout(location = 5) in vec3 normal;
layout(location = 6) in vec3 eye_vec;
layout(location = 7) in vec3 frag_pos;

//vec3 light_pos = vec3(100, 100, 100);

float blinnCorr = 8.0;

void main() {
  vec3 final_color = color_ambient;

  vec3 N = normalize(normal);
  vec3 L = normalize(lightSettings.light_pos - eye_vec);
  vec3 E = normalize(-eye_vec);
  vec3 V = normalize(eye_vec - frag_pos); // view direction
  vec3 H = normalize(L+V); // halfway vector
  float lambertTerm = dot(N, L);

  if (lambertTerm > 0.0) {
    final_color += color_diffuse * lambertTerm;
    vec3 R = reflect(-L, N);
    float specular = pow(max(dot(N, H), 0.0), blinnCorr+shininess);
    final_color += color_specular * specular;
  }

  FragColor = vec4(final_color, opacity);
}
