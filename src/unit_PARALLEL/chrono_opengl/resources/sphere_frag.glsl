#version 330

layout(location = 0) out vec4 FragColor;
uniform vec4 color;

void main() {
  const vec3 lightDir = vec3(0.577, 0.577, 0.577);
  vec3 N;
  N.xy = gl_PointCoord.xy - 0.5;

  float mag = dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5);

  if (mag > 0.25) {
    discard;
  } else {
    N.z = sqrt(1.0 - mag * 4);
    float diffuse = max(0.0, dot(lightDir, N));
    FragColor = vec4(color.xyz * diffuse, 1);
  }
}
