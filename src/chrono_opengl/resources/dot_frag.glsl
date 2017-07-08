#version 330

layout(location = 0) out vec4 FragColor;
uniform vec4 color;

void main() {
  float d = dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) * 4;
  if (d > 1) {
    discard;
  } else if (d > .9 && d < 1) {
    FragColor = vec4(0, 0, 0, 1);
  } else {
    FragColor = vec4(color.xyz, 1);
  }
}
