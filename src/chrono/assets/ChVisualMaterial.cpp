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
// Authors: Asher Elmquist, Radu Serban
// =============================================================================

#include "chrono/assets/ChVisualMaterial.h"

namespace chrono {

ChVisualMaterial::ChVisualMaterial()
    : Ka({0.2f, 0.2f, 0.2f}),
      Kd({0.5f, 0.5f, 0.5f}),
      Ks({0.2f, 0.2f, 0.2f}),
      Ns(88),
      d(1),
      fresnel_exp(5),
      fresnel_max(1),
      fresnel_min(0.f),
      roughness(1),
      metallic(0),
      use_specular_workflow(true),
      class_id(0),
      instance_id(0) {}

void ChVisualMaterial::SetAmbientColor(const ChVector<float>& rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Ka = rgb;
    }
}
void ChVisualMaterial::SetDiffuseColor(const ChVector<float>& rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Kd = rgb;
    }
}
void ChVisualMaterial::SetSpecularColor(const ChVector<float>& rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Ks = rgb;
    }
}
void ChVisualMaterial::SetEmissiveColor(const ChVector<float>& rgb) {
    // valid rgb range [0,1]
    if (rgb.x() >= 0 && rgb.y() >= 0 && rgb.z() >= 0 && rgb.x() <= 1 && rgb.y() <= 1 && rgb.z() <= 1) {
        Ke = rgb;
    }
}

void ChVisualMaterial::SetSpecularExponent(float exponent) {
    // valid exponent range [0,1000]
    if (exponent >= 0 && exponent <= 1000) {
        Ns = exponent;
    }
}

void ChVisualMaterial::SetOpacity(float o) {
    // valid transparent range [0,1] 1=opaque, 0=transparent
    if (o >= 0 && o <= 1) {
        d = o;
    }
}

void ChVisualMaterial::SetIllumination(int i) {
    // valid values for illumination model are 0...10
    if (i >= 0 && i <= 10) {
        illum = i;
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

void ChVisualMaterial::SetMetallic(float m) {
    metallic = std::max(0.001f, std::min(m, 1.f));
}

// -----------------------------------------------------------------------------

static std::shared_ptr<ChVisualMaterial> default_material;

std::shared_ptr<ChVisualMaterial> ChVisualMaterial::Default() {
    if (!default_material) {
        default_material = chrono_types::make_shared<ChVisualMaterial>();
        default_material->SetDiffuseColor(ChVector<float>(1, 1, 1));
    }

    return default_material;
}

}  // end namespace chrono
