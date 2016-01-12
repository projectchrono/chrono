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

/// @addtogroup opengl
/// @{

static ChOpenGLMaterial white(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1), glm::vec3(1, 1, 1));
static ChOpenGLMaterial red(glm::vec3(0, 0, 0), glm::vec3(1, 0, 0), glm::vec3(1, 1, 1));
static ChOpenGLMaterial river(glm::vec3(0, 0, 0),
                              glm::vec3(52.0 / 255.0, 152.0 / 255.0, 219.0 / 255.0),
                              glm::vec3(1, 1, 1));
static ChOpenGLMaterial darkriver(glm::vec3(0, 0, 0),
                                  glm::vec3(77.0 / 255.0, 114.0 / 255.0, 130.0 / 255.0),
                                  glm::vec3(1, 1, 1));
static ChOpenGLMaterial brightriver(glm::vec3(0, 0, 0),
                                    glm::vec3(146.0 / 255.0, 214.0 / 255.0, 255.0 / 255.0),
                                    glm::vec3(1, 1, 1));

static float ambient = .5;

static ChOpenGLMaterial slate(glm::vec3(85.0f, 98.0f, 112.0f) / 255.0f * ambient,
                              glm::vec3(85.0f, 98.0f, 112.0f) / 255.0f,
                              glm::vec3(1, 1, 1));
static ChOpenGLMaterial pacifica(glm::vec3(78.0f, 205.0f, 196.0f) / 255.0f * ambient,
                                 glm::vec3(78.0f, 205.0f, 196.0f) / 255.0f,
                                 glm::vec3(1, 1, 1));
static ChOpenGLMaterial apple(glm::vec3(199.0f, 244.0f, 100.0f) / 255.0f * ambient,
                              glm::vec3(199.0f, 244.0f, 100.0f) / 255.0f,
                              glm::vec3(1, 1, 1));
static ChOpenGLMaterial cherry(glm::vec3(255.0f, 107.0f, 107.0f) / 255.0f * ambient,
                               glm::vec3(255.0f, 107.0f, 107.0f) / 255.0f,
                               glm::vec3(1, 1, 1));
static ChOpenGLMaterial pillow(glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f * ambient,
                               glm::vec3(196.0f, 77.0f, 88.0f) / 255.0f,
                               glm::vec3(1, 1, 1));

static ChOpenGLMaterial elated(glm::vec3(255.0f, 171.0f, 25.0f) / 255.0f * ambient,
                               glm::vec3(255.0f, 171.0f, 25.0f) / 255.0f,
                               glm::vec3(1, 1, 1));
static ChOpenGLMaterial greyslate(glm::vec3(158.0f, 158.0f, 158.0f) / 255.0f * ambient,
                                  glm::vec3(158.0f, 158.0f, 158.0f) / 255.0f,
                                  glm::vec3(1, 1, 1));
static ChOpenGLMaterial darkred(glm::vec3(193.0f, 21.0f, 21.0f) / 255.0f * ambient,
                                glm::vec3(193.0f, 21.0f, 21.0f) / 255.0f,
                                glm::vec3(1, 1, 1));

static ChOpenGLMaterial t1(glm::vec3(236.0f, 208.0f, 120.0f) / 255.0f * ambient,
                           glm::vec3(236.0f, 208.0f, 120.0f) / 255.0f,
                           glm::vec3(1, 1, 1));
static ChOpenGLMaterial t2(glm::vec3(217.0f, 91.0f, 67.0f) / 255.0f * ambient,
                           glm::vec3(217.0f, 91.0f, 67.0f) / 255.0f,
                           glm::vec3(1, 1, 1));
static ChOpenGLMaterial t3(glm::vec3(192.0f, 41.0f, 66.0f) / 255.0f * ambient,
                           glm::vec3(192.0f, 41.0f, 66.0f) / 255.0f,
                           glm::vec3(1, 1, 1));
static ChOpenGLMaterial t4(glm::vec3(84.0f, 36.0f, 55.0f) / 255.0f * ambient,
                           glm::vec3(84.0f, 36.0f, 55.0f) / 255.0f,
                           glm::vec3(1, 1, 1));
static ChOpenGLMaterial t5(glm::vec3(83.0f, 119.0f, 122.0f) / 255.0f * ambient,
                           glm::vec3(83.0f, 119.0f, 122.0f) / 255.0f,
                           glm::vec3(1, 1, 1));

static ChOpenGLMaterial text_mat(glm::vec3(0, 0, 0), glm::vec3(100.0f, 145.0f, 170.0f) / 255.0f, glm::vec3(1, 1, 1));

/// @} opengl

}
}
#endif  // END of CHOPENGLMATERIALS_H
