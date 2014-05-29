#version 410 core

uniform sampler2D tex;
in vec2 texCoords;
out vec4 fragColor;
const vec4 color = vec4(1, 1, 1, 1);

void main(void) {
    fragColor = vec4(1, 1, 1, texture(tex, texCoords).r) * color;
    //fragColor = vec4(1, 1, 1, 1);
}