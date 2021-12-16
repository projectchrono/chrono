#version 300 es

precision highp float;

layout(location = 0) out vec4 FragColor;
flat in vec3 color;
void main() {
  FragColor = vec4(color, 1);
}
