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
//
// Visual assets that can be used for higher quality rendering such as that
// used by the sensing module. These materials follow, in part, from the
// Wavefront obj mtl specification.
//
// =============================================================================

#ifndef CHVISUALMATERIAL_H
#define CHVISUALMATERIAL_H

#include "chrono/assets/ChAsset.h"

namespace chrono {

/// Visual material.
class ChApi ChVisualMaterial : public ChAsset {
  public:
    ChVisualMaterial();

    // Setting functions
    void SetAmbientColor(const ChVector<float>& rgb);
    void SetDiffuseColor(const ChVector<float>& rgb);
    void SetSpecularColor(const ChVector<float>& rgb);
    void SetEmissiveColor(const ChVector<float>& rgb);
    void SetSpecularExponent(float exponent);
    void SetTransparency(float tr);
    void SetIllumination(int i);
    void SetKdTexture(const std::string& filename) { kd_texture = filename; };
    void SetKsTexture(const std::string& filename) { ks_texture = filename; };
    void SetNormalMapTexture(const std::string& filename) { normal_texture = filename; };
    void SetMetallicTexture(const std::string& filename) { metallic_texture = filename; };
    void SetRoughnessTexture(const std::string& filename) { roughness_texture = filename; };
    void SetOpacityTexture(const std::string& filename) { opacity_texture = filename; };
    void SetFresnelExp(float exp);
    void SetFresnelMax(float max);
    void SetFresnelMin(float min);
    void SetRoughness(float r);
    void SetMetallic(float m);
    void SetUseSpecularWorkflow(bool s) { use_specular_workflow = s; }
    void SetClassID(unsigned short int id) { class_id = id; }
    void SetInstanceID(unsigned short int id) { instance_id = id; }

    // accessor functions
    const ChVector<float>& GetAmbientColor() const { return Ka; }
    const ChVector<float>& GetDiffuseColor() const { return Kd; }
    const ChVector<float>& GetSpecularColor() const { return Ks; }
    const ChVector<float>& GetEmissiveColor() const { return Ke; }
    float GetSpecularExponent() const { return Ns; }
    float GetTransparency() const { return d; }
    int GetIllumination() const { return illum; }
    const std::string& GetKdTexture() const { return kd_texture; };
    const std::string& GetKsTexture() const { return ks_texture; };
    const std::string& GetNormalMapTexture() const { return normal_texture; };
    const std::string& GetMetallicTexture() const { return metallic_texture; };
    const std::string& GetRoughnessTexture() const { return roughness_texture; };
    const std::string& GetOpacityTexture() const { return opacity_texture; };
    float GetFresnelExp() const { return fresnel_exp; }
    float GetFresnelMax() const { return fresnel_max; }
    float GetFresnelMin() const { return fresnel_min; }
    float GetRoughness() const { return roughness; }
    float GetMetallic() const { return metallic; }
    bool GetUseSpecularWorkflow() const { return use_specular_workflow; }
    unsigned short int GetClassID() const { return class_id; }
    unsigned short int GetInstanceID() const { return instance_id; }

  private:
    ChVector<float> Ka;  // ambient color 0-1
    ChVector<float> Kd;  // diffuse color 0-1
    ChVector<float> Ks;  // specular color 0-1
    ChVector<float> Ke;  // emissive color 0-1

    float fresnel_max;
    float fresnel_min;
    float fresnel_exp;
    float Ns;  // specular exponent
    float d;   // transparency

    int illum;  // illumination model (see http://www.fileformat.info/format/material/)

    float roughness;
    float metallic;

    bool use_specular_workflow;

    std::string kd_texture;
    std::string ks_texture;
    std::string normal_texture;
    std::string metallic_texture;
    std::string roughness_texture;
    std::string opacity_texture;

    unsigned short int class_id;
    unsigned short int instance_id;
};

}  // end namespace chrono

#endif
