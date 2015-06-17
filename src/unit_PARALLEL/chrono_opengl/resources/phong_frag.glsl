#version 330

layout(location = 0) out vec4 FragColor;

// uniform mat4 mvp;
// uniform mat4 modelview_matrix;
// uniform mat3 normal_matrix;

float shininess = 100.0;

flat in vec3 color_ambient;
flat in vec3 color_diffuse;
vec3 color_specular = vec3(1, 1, 1);

vec3 light_pos = vec3(100, 100, 100);
in vec3 normal;
in vec3 eye_vec;

void main() {
  vec3 final_color = color_ambient;

  vec3 N = normalize(normal);
  vec3 L = normalize(light_pos - eye_vec);
  vec3 E = normalize(-eye_vec);

  float lambertTerm = dot(N, L);

  if (lambertTerm > 0.0) {
    final_color += color_diffuse * lambertTerm;
    vec3 R = reflect(-L, N);
    float specular = pow(max(dot(R, E), 0.0), shininess);
    final_color += color_specular * specular;
  }

  FragColor = vec4(final_color, 1);
}
