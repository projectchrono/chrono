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

#include "chrono_opengl/core/ChOpenGLVertexAttributes.h"

using namespace glm;

namespace chrono {
namespace opengl {

ChOpenGLVertexAttributesPADSNT::ChOpenGLVertexAttributesPADSNT() {
    this->position = vec3(0.0f);
    this->color_ambient = vec3(0.0f);
    this->color_diffuse = vec3(0.0f);
    this->color_specular = vec3(0.0f);
    this->normal = vec3(0.0f);
    this->texture_coordinate = vec2(0.0f);
}
ChOpenGLVertexAttributesPADSNT::ChOpenGLVertexAttributesPADSNT(const glm::vec3& p,
                                                               const glm::vec3& c_a,
                                                               const glm::vec3& c_d,
                                                               const glm::vec3& c_s,
                                                               const glm::vec3& n,
                                                               const glm::vec2& t) {
    this->position = p;
    this->color_ambient = c_a;
    this->color_diffuse = c_d;
    this->color_specular = c_s;
    this->normal = n;
    this->texture_coordinate = t;
}
ChOpenGLVertexAttributesPADSNT::ChOpenGLVertexAttributesPADSNT(const ChOpenGLVertexAttributesPADSNT& other) {
    this->position = other.position;
    this->color_ambient = other.color_ambient;
    this->color_diffuse = other.color_diffuse;
    this->color_specular = other.color_specular;
    this->normal = other.normal;
    this->texture_coordinate = other.texture_coordinate;
}

ChOpenGLVertexAttributesPADSN::ChOpenGLVertexAttributesPADSN() {
    this->position = vec3(0.0f);
    this->color_ambient = vec3(0.0f);
    this->color_diffuse = vec3(0.0f);
    this->color_specular = vec3(0.0f);
    this->normal = vec3(0.0f);
}
ChOpenGLVertexAttributesPADSN::ChOpenGLVertexAttributesPADSN(const glm::vec3& p,
                                                             const glm::vec3& c_a,
                                                             const glm::vec3& c_d,
                                                             const glm::vec3& c_s,
                                                             const glm::vec3& n) {
    this->position = p;
    this->color_ambient = c_a;
    this->color_diffuse = c_d;
    this->color_specular = c_s;
    this->normal = n;
}
ChOpenGLVertexAttributesPADSN::ChOpenGLVertexAttributesPADSN(const ChOpenGLVertexAttributesPADSN& other) {
    this->position = other.position;
    this->color_ambient = other.color_ambient;
    this->color_diffuse = other.color_diffuse;
    this->color_specular = other.color_specular;
    this->normal = other.normal;
}

ChOpenGLVertexAttributesPCNT::ChOpenGLVertexAttributesPCNT() {
    this->position = vec3(0.0f);
    this->color = vec3(0.0f);
    this->normal = vec3(0.0f);
    this->texture_coordinate = vec2(0.0f);
}

ChOpenGLVertexAttributesPCNT::ChOpenGLVertexAttributesPCNT(const vec3& p, const vec3& c, const vec3& n, const vec2& t) {
    this->position = p;
    this->color = c;
    this->normal = n;
    this->texture_coordinate = t;
}

ChOpenGLVertexAttributesPCNT::ChOpenGLVertexAttributesPCNT(const ChOpenGLVertexAttributesPCNT& other) {
    this->position = other.position;
    this->color = other.color;
    this->normal = other.normal;
    this->texture_coordinate = other.texture_coordinate;
}

ChOpenGLVertexAttributesP::ChOpenGLVertexAttributesP(const vec3& p) {
    this->position = p;
}

ChOpenGLVertexAttributesP::ChOpenGLVertexAttributesP(const ChOpenGLVertexAttributesP& other) {
    this->position = other.position;
}

ChOpenGLVertexAttributesP::ChOpenGLVertexAttributesP() {
    this->position = vec3(0.0f);
}

ChOpenGLVertexAttributesPN::ChOpenGLVertexAttributesPN(const vec3& p, const vec3& n) {
    this->position = p;
    this->normal = n;
}

ChOpenGLVertexAttributesPN::ChOpenGLVertexAttributesPN(const ChOpenGLVertexAttributesPN& other) {
    this->position = other.position;
    this->normal = other.normal;
}

ChOpenGLVertexAttributesPN::ChOpenGLVertexAttributesPN() {
    this->position = vec3(0.0f);
    this->normal = vec3(0.0f, 0.0f, 1.0f);
}

ChOpenGLVertexAttributesPCN::ChOpenGLVertexAttributesPCN() {
    this->position = vec3(0.0f);
    this->color = vec3(0.0f);
    this->normal = vec3(0.0f);
}

ChOpenGLVertexAttributesPCN::ChOpenGLVertexAttributesPCN(const vec3& p, const vec3& c, const vec3& n) {
    this->position = p;
    this->color = c;
    this->normal = n;
}

ChOpenGLVertexAttributesPCN::ChOpenGLVertexAttributesPCN(const ChOpenGLVertexAttributesPCN& other) {
    this->position = other.position;
    this->color = other.color;
    this->normal = other.normal;
}
}
}
