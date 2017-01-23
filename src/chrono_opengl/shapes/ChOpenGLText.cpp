// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Hammad Mazhar
// =============================================================================
// Generic renderable text class that uses an atlas stored in the FontData.h
// file. Add text and once finished run the draw command.
// =============================================================================

#include <iostream>
#include <glm/gtc/type_ptr.hpp>

#include "chrono_opengl/shapes/ChOpenGLText.h"
#include "chrono_opengl/FontData.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLText::ChOpenGLText() : ChOpenGLObject() {
    texture = BAD_GL_VALUE;
    sampler = BAD_GL_VALUE;
    vao = BAD_GL_VALUE;
    vbo = BAD_GL_VALUE;
    texture_handle = BAD_GL_VALUE;
    color_handle = BAD_GL_VALUE;
}

bool ChOpenGLText::Initialize(ChOpenGLMaterial mat, ChOpenGLShader* _shader) {
    if (GLReturnedError("Background::Initialize - on entry"))
        return false;

    if (!super::Initialize()) {
        return false;
    }

    glGenBuffers(1, &vbo);
    glGenVertexArrays(1, &vao);
    glGenTextures(1, &texture);
    glGenSamplers(1, &sampler);
    glSamplerParameteri(sampler, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glSamplerParameteri(sampler, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glSamplerParameteri(sampler, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glSamplerParameteri(sampler, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // get the uniform location for the texture from shader

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, (GLsizei)font_data.tex_width, (GLsizei)font_data.tex_height, 0, GL_RED,
                 GL_UNSIGNED_BYTE, font_data.tex_data);
    glBindTexture(GL_TEXTURE_2D, 0);

    this->AttachShader(_shader);
    texture_handle = this->shader->GetUniformLocation("tex");
    color_handle = this->shader->GetUniformLocation("color");
    color = glm::vec4(mat.diffuse_color, 1);

    GenerateFontIndex();
    text_data.reserve(1000);

    return true;
}
void ChOpenGLText::Update() {
    text_data.clear();
}

void ChOpenGLText::GenerateFontIndex() {
    std::string chars =
        " !\"#$%&'()*+,-./"
        "0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`"
        "abcdefghijklmnopqrstuvwxyz{|}~";

    for (int i = 0; i < chars.size(); i++) {
        for (int j = 0; j < font_data.glyphs_count; ++j) {
            if (font_data.glyphs[j].charcode == chars[i]) {
                char_index[chars[i]] = j;
                break;
            }
        }
    }
}

void ChOpenGLText::Render(const std::string& str, float x, float y, float sx, float sy) {
    for (int i = 0; i < str.size(); i++) {
        texture_glyph_t* glyph = 0;
        glyph = &font_data.glyphs[char_index[str[i]]];

        if (!glyph) {
            continue;
        }
        x += glyph->kerning[0].kerning;
        float x0 = (float)(x + glyph->offset_x * sx);
        float y0 = (float)(y + glyph->offset_y * sy);
        float x1 = (float)(x0 + glyph->width * sx);
        float y1 = (float)(y0 - glyph->height * sy);

        float s0 = glyph->s0;
        float t0 = glyph->t0;
        float s1 = glyph->s1;
        float t1 = glyph->t1;

        text_data.push_back(glm::vec4(x0, y0, s0, t0));
        text_data.push_back(glm::vec4(x0, y1, s0, t1));
        text_data.push_back(glm::vec4(x1, y1, s1, t1));
        text_data.push_back(glm::vec4(x0, y0, s0, t0));
        text_data.push_back(glm::vec4(x1, y1, s1, t1));
        text_data.push_back(glm::vec4(x1, y0, s1, t0));

        x += (glyph->advance_x * sx);
    }
}

void ChOpenGLText::TakeDown() {
    super::TakeDown();
}

void ChOpenGLText::Draw(const mat4& projection, const mat4& view) {
    if (GLReturnedError("ChOpenGLText::Draw - on entry"))
        return;

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, texture);

    glBindSampler(0, sampler);
    glBindVertexArray(vao);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // Enable the shader
    shader->Use();

    glUniform1i(texture_handle, 0);
    glUniform3fv(color_handle, 1, glm::value_ptr(color));
    glBufferData(GL_ARRAY_BUFFER, text_data.size() * sizeof(glm::vec4), &this->text_data[0], GL_STATIC_DRAW);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)text_data.size());
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);

    if (GLReturnedError("ChOpenGLText::Draw - on exit"))
        return;
}
}
}
