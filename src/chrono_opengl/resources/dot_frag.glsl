#version 300 es

precision highp float;

layout(location = 0) out vec4 FragColor;
uniform vec4 color;

void main() {
  float d = dot(gl_PointCoord - 0.5, gl_PointCoord - 0.5) * 4.0;
  if (d > 1.0) {
    discard;
  } else if (d > .9 && d < 1.0) {
    FragColor = vec4(0, 0, 0, 1);
  } else {
    FragColor = vec4(color.xyz, 1);
  }
}
