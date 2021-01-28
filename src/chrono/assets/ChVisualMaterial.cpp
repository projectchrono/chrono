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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/assets/ChVisualMaterial.h"

namespace chrono {

ChVisualMaterial::ChVisualMaterial()
    : Ka({0.1f, 0.1f, 0.1f}),
      Kd({0.5f, 0.5f, 0.5f}),
      Ks({0.2f, 0.2f, 0.2f}),
      Ns(88.0f),
      d(1.0f),
      fresnel_exp(5.0f),
      fresnel_max(1.0f),
      fresnel_min(0.0f),
      roughness(0.5f) {}

void ChVisualMaterial::SetAmbientColor(ChVector<float> rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Ka = rgb;
    }
}
void ChVisualMaterial::SetDiffuseColor(ChVector<float> rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Kd = rgb;
    }
}
void ChVisualMaterial::SetSpecularColor(ChVector<float> rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Ks = rgb;
    }
}

void ChVisualMaterial::SetSpecularExponent(float exponent) {
    // valid exponent range [0,1000]
    if (exponent >= 0 && exponent <= 1000) {
        Ns = exponent;
    }
}

void ChVisualMaterial::SetTransparency(float tr) {
    // valid transparent range [0,1] 1=opaque, 0=transparent
    if (tr >= 0 && tr <= 1) {
        d = tr;
    }
}

void ChVisualMaterial::SetFresnelExp(float exp) {
    fresnel_exp = std::max(1.f, std::min(exp, 1000.f));
}
void ChVisualMaterial::SetFresnelMax(float max) {
    fresnel_max = std::max(0.f, std::min(max, 1.f));
}
void ChVisualMaterial::SetFresnelMin(float min) {
    fresnel_min = std::max(0.f, std::min(min, 1.f));
}

void ChVisualMaterial::SetRoughness(float r) {
    roughness = std::max(0.001f, std::min(r, 1.f));
}

}  // end namespace chrono
