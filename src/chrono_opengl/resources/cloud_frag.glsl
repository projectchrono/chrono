#version 300 es

precision highp float;

layout(location = 0) out vec4 FragColor;
uniform vec4 color;

void main() {
  FragColor = color;
}