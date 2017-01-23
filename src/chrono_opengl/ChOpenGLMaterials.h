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
// Shared materials used in the OpenGL viewer
// =============================================================================

#ifndef CHOPENGLMATERIALS_H
#define CHOPENGLMATERIALS_H

#include "chrono_opengl/core/ChOpenGLMaterial.h"

namespace chrono {
namespace opengl {

static glm::vec3 ColorConverter(int hex_value) {
    glm::vec3 rgbColor;

    rgbColor.r = ((hex_value >> 16) & 0xFF) / 255.0f;  // Extract the RR byte
    rgbColor.g = ((hex_value >> 8) & 0xFF) / 255.0f;   // Extract the GG byte
    rgbColor.b = ((hex_value)&0xFF) / 255.0f;          // Extract the BB byte

    return rgbColor;
}

/// @addtogroup opengl
/// @{

static ChOpenGLMaterial white(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
static ChOpenGLMaterial red(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 1, 1));

static ChOpenGLMaterial grid(glm::vec3(0, 0, 0), ColorConverter(0x324D5C), glm::vec3(1, 1, 1));

static ChOpenGLMaterial sphere_color(glm::vec3(0, 0, 0), ColorConverter(0xDE5B49), glm::vec3(1, 1, 1));
static ChOpenGLMaterial box_color(glm::vec3(0, 0, 0), ColorConverter(0xE37B40), glm::vec3(1, 1, 1));
static ChOpenGLMaterial cylinder_color(glm::vec3(0, 0, 0), ColorConverter(0xF0CA4D), glm::vec3(1, 1, 1));
static ChOpenGLMaterial cone_color(glm::vec3(0, 0, 0), ColorConverter(0x46B29D), glm::vec3(1, 1, 1));
static ChOpenGLMaterial mesh_color(glm::vec3(0, 0, 0), ColorConverter(0x3498DB), glm::vec3(1, 1, 1));
static ChOpenGLMaterial fea_color(glm::vec3(0, 0, 0), ColorConverter(0x00A388), glm::vec3(1, 1, 1));
static ChOpenGLMaterial mpm_node_color(glm::vec3(0, 0, 0), ColorConverter(0xBEEB9F), glm::vec3(1, 1, 1));

static ChOpenGLMaterial contact_color(glm::vec3(0, 0, 0), ColorConverter(0xFFF0A5), glm::vec3(1, 1, 1));

static ChOpenGLMaterial text_mat(glm::vec3(0, 0, 0), glm::vec3(100.0f, 145.0f, 170.0f) / 255.0f, glm::vec3(1, 1, 1));

/// @} opengl
}
}
#endif
