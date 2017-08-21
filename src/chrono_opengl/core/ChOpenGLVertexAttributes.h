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
// This class supports ADS lighting with glow and texture coordinates
// Nothing too interesting here, note that the order for the members is important
// Based on code provided by Perry Kivolowitz.
// P = position
// C = color
// N = normal
// T = texture coordinates

// A = Ambient
// D = Diffuse
// S = Specular
// =============================================================================

#ifndef CHOPENGLVERTEXATTRIBUTE_H
#define CHOPENGLVERTEXATTRIBUTE_H

#include "chrono_opengl/core/ChApiOpenGL.h"
#include <glm/glm.hpp>

namespace chrono {
namespace opengl {

/// @addtogroup opengl_module
/// @{

/// Support for ADS lighting with glow and texture coordinates.
class CH_OPENGL_API ChOpenGLVertexAttributesPADSNT {
  public:
    ChOpenGLVertexAttributesPADSNT();
    ChOpenGLVertexAttributesPADSNT(const glm::vec3& p,
                                   const glm::vec3& c_a,
                                   const glm::vec3& c_d,
                                   const glm::vec3& c_s,
                                   const glm::vec3& n,
                                   const glm::vec2& t);
    ChOpenGLVertexAttributesPADSNT(const ChOpenGLVertexAttributesPADSNT& other);
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color_ambient;
    glm::vec3 color_diffuse;
    glm::vec3 color_specular;
    glm::vec2 texture_coordinate;
};

/// Support for ADS lighting with glow.
class CH_OPENGL_API ChOpenGLVertexAttributesPADSN {
  public:
    ChOpenGLVertexAttributesPADSN();
    ChOpenGLVertexAttributesPADSN(const glm::vec3& p,
                                  const glm::vec3& c_a,
                                  const glm::vec3& c_d,
                                  const glm::vec3& c_s,
                                  const glm::vec3& n);
    ChOpenGLVertexAttributesPADSN(const ChOpenGLVertexAttributesPADSN& other);
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color_ambient;
    glm::vec3 color_diffuse;
    glm::vec3 color_specular;
};

class CH_OPENGL_API ChOpenGLVertexAttributesPCNT {
  public:
    ChOpenGLVertexAttributesPCNT();
    ChOpenGLVertexAttributesPCNT(const glm::vec3& p, const glm::vec3& c, const glm::vec3& n, const glm::vec2& t);
    ChOpenGLVertexAttributesPCNT(const ChOpenGLVertexAttributesPCNT& other);
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
    glm::vec2 texture_coordinate;
};

class CH_OPENGL_API ChOpenGLVertexAttributesPCN {
  public:
    ChOpenGLVertexAttributesPCN();
    ChOpenGLVertexAttributesPCN(const glm::vec3& p, const glm::vec3& c, const glm::vec3& n);
    ChOpenGLVertexAttributesPCN(const ChOpenGLVertexAttributesPCN& other);
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec3 color;
};

class CH_OPENGL_API ChOpenGLVertexAttributesPN {
  public:
    ChOpenGLVertexAttributesPN();
    ChOpenGLVertexAttributesPN(const glm::vec3& p, const glm::vec3& n);
    ChOpenGLVertexAttributesPN(const ChOpenGLVertexAttributesPN& other);
    glm::vec3 position;
    glm::vec3 normal;
};

class CH_OPENGL_API ChOpenGLVertexAttributesP {
  public:
    ChOpenGLVertexAttributesP();
    ChOpenGLVertexAttributesP(const glm::vec3& p);
    ChOpenGLVertexAttributesP(const ChOpenGLVertexAttributesP& other);
    glm::vec3 position;
};

/// @} opengl_module

}
}

#endif
