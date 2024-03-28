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

CH_FACTORY_REGISTER(ChVisualMaterial)

ChVisualMaterial::ChVisualMaterial()
    : Ka({0.2f, 0.2f, 0.2f}),
      Kd({1.0f, 1.0f, 1.0f}),
      Ks({0.2f, 0.2f, 0.2f}),
      Ke({0.0f, 0.0f, 0.0f}),
      Ns(88),
      d(1),
      fresnel_exp(5),
      fresnel_max(1),
      fresnel_min(0.f),
      illum(0),
      roughness(0),
      metallic(0),
      anisotropy(0.f),
      use_specular_workflow(true),
      class_id(0),
      instance_id(0),
      use_hapke(false),
      emissive_power(0.f) {}

void ChVisualMaterial::SetKdTexture(const std::string& filename) {
    kd_texture.SetFilename(filename);
}
void ChVisualMaterial::SetKsTexture(const std::string& filename) {
    ks_texture.SetFilename(filename);
}
void ChVisualMaterial::SetKeTexture(const std::string& filename) {
    ke_texture.SetFilename(filename);
}
void ChVisualMaterial::SetNormalMapTexture(const std::string& filename) {
    normal_texture.SetFilename(filename);
}
void ChVisualMaterial::SetMetallicTexture(const std::string& filename) {
    metallic_texture.SetFilename(filename);
}
void ChVisualMaterial::SetRoughnessTexture(const std::string& filename) {
    roughness_texture.SetFilename(filename);
}
void ChVisualMaterial::SetOpacityTexture(const std::string& filename) {
    opacity_texture.SetFilename(filename);
}
void ChVisualMaterial::SetWeightTexture(const std::string& filename) {
    weight_texture.SetFilename(filename);
}
void ChVisualMaterial::SetDisplacementTexture(const std::string& filename) {
    disp_texture.SetFilename(filename);
}
void ChVisualMaterial::SetAmbientOcclusionTexture(const std::string& filename) {
    ao_texture.SetFilename(filename);
}

void ChVisualMaterial::SetTextureScale(float scale_x, float scale_y) {
    kd_texture.SetScale(scale_x, scale_y);
    ks_texture.SetScale(scale_x, scale_y);
    ke_texture.SetScale(scale_x, scale_y);
    normal_texture.SetScale(scale_x, scale_y);
    metallic_texture.SetScale(scale_x, scale_y);
    roughness_texture.SetScale(scale_x, scale_y);
    opacity_texture.SetScale(scale_x, scale_y);
    weight_texture.SetScale(scale_x, scale_y);
    disp_texture.SetScale(scale_x, scale_y);
    ao_texture.SetScale(scale_x, scale_y);
}

const ChVector2f& ChVisualMaterial::GetTextureScale() const {
    return kd_texture.GetScale();
}

void ChVisualMaterial::SetAmbientColor(const ChColor& rgb) {
    // valid rgb range [0,1]
    if (rgb.R >= 0 && rgb.G >= 0 && rgb.B >= 0 && rgb.R <= 1 && rgb.G <= 1 && rgb.B <= 1) {
        Ka = rgb;
    }
}
void ChVisualMaterial::SetDiffuseColor(const ChColor& rgb) {
    // valid rgb range [0,1]
    if (rgb.R >= 0 && rgb.G >= 0 && rgb.B >= 0 && rgb.R <= 1 && rgb.G <= 1 && rgb.B <= 1) {
        Kd = rgb;
    }
}
void ChVisualMaterial::SetSpecularColor(const ChColor& rgb) {
    // valid rgb range [0,1]
    if (rgb.R >= 0 && rgb.G >= 0 && rgb.B >= 0 && rgb.R <= 1 && rgb.G <= 1 && rgb.B <= 1) {
        Ks = rgb;
    }
}
void ChVisualMaterial::SetEmissiveColor(const ChColor& rgb) {
    // valid rgb range [0,1]
    if (rgb.R >= 0 && rgb.G >= 0 && rgb.B >= 0 && rgb.R <= 1 && rgb.G <= 1 && rgb.B <= 1) {
        Ke = rgb;
    }
}

void ChVisualMaterial::SetEmissivePower(const float& power) {
    emissive_power = power;
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

void ChVisualMaterial::SetAnisotropy(float a){
    anisotropy = std::max(0.f, std::min(a, 1.f));
}

void ChVisualMaterial::SetHapkeParameters(float w, float b, float c, float B_s0, float h_s, float phi, float theta_p) {
    hapke_w = w;
    hapke_b = b;
    hapke_c = c;
    hapke_B_s0 = B_s0;
    hapke_h_s = h_s;
    hapke_phi = phi;
    hapke_theta_p = theta_p;
}

void ChVisualMaterial::ArchiveOut(ChArchiveOut& archive_out) {
    archive_out.VersionWrite<ChVisualMaterial>();

    archive_out << CHNVP(Ka);
    archive_out << CHNVP(Kd);
    archive_out << CHNVP(Ks);
    archive_out << CHNVP(Ke);
    archive_out << CHNVP(fresnel_max);
    archive_out << CHNVP(fresnel_min);
    archive_out << CHNVP(fresnel_exp);
    archive_out << CHNVP(Ns);
    archive_out << CHNVP(d);
    archive_out << CHNVP(illum);
    archive_out << CHNVP(roughness);
    archive_out << CHNVP(metallic);
    archive_out << CHNVP(use_specular_workflow);
    archive_out << CHNVP(kd_texture);
    archive_out << CHNVP(ks_texture);
    archive_out << CHNVP(ke_texture);
    archive_out << CHNVP(normal_texture);
    archive_out << CHNVP(metallic_texture);
    archive_out << CHNVP(roughness_texture);
    archive_out << CHNVP(opacity_texture);
    archive_out << CHNVP(weight_texture);
    archive_out << CHNVP(disp_texture);
    archive_out << CHNVP(ao_texture);
}

void ChVisualMaterial::ArchiveIn(ChArchiveIn& archive_in) {
    /*int version =*/archive_in.VersionRead<ChVisualMaterial>();

    archive_in >> CHNVP(Ka);
    archive_in >> CHNVP(Kd);
    archive_in >> CHNVP(Ks);
    archive_in >> CHNVP(Ke);
    archive_in >> CHNVP(fresnel_max);
    archive_in >> CHNVP(fresnel_min);
    archive_in >> CHNVP(fresnel_exp);
    archive_in >> CHNVP(Ns);
    archive_in >> CHNVP(d);
    archive_in >> CHNVP(illum);
    archive_in >> CHNVP(roughness);
    archive_in >> CHNVP(metallic);
    archive_in >> CHNVP(use_specular_workflow);
    archive_in >> CHNVP(kd_texture);
    archive_in >> CHNVP(ks_texture);
    archive_in >> CHNVP(ke_texture);
    archive_in >> CHNVP(normal_texture);
    archive_in >> CHNVP(metallic_texture);
    archive_in >> CHNVP(roughness_texture);
    archive_in >> CHNVP(opacity_texture);
    archive_in >> CHNVP(weight_texture);
    archive_in >> CHNVP(disp_texture);
    archive_in >> CHNVP(ao_texture);
}

// -----------------------------------------------------------------------------

static std::shared_ptr<ChVisualMaterial> default_material;

std::shared_ptr<ChVisualMaterial> ChVisualMaterial::Default() {
    if (!default_material) {
        default_material = chrono_types::make_shared<ChVisualMaterial>();
    }

    return default_material;
}

}  // end namespace chrono
