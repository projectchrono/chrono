#version 330

uniform sampler2D tex;
uniform vec3 color;

in vec2 texCoords;
out vec4 fragColor;

void main(void) {
  fragColor = vec4(color, texture(tex, texCoords).r);
}
