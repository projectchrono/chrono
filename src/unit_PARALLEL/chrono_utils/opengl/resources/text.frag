#version 330

uniform sampler2D tex;
in vec2 texCoords;
out vec4 fragColor;

void main(void) {
    fragColor = vec4(1, 1, 1, texture(tex, texCoords).r);
}