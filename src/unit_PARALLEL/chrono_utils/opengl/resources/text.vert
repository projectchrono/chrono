#version 410 core

in vec4 position;
out vec2 texCoords;

void main(void) {
    gl_Position = vec4(position.xy, 0, 1);
    texCoords = position.zw;
}