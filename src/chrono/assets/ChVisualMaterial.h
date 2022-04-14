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

#include <string>

#include "chrono/core/ChVector.h"
#include "chrono/assets/ChTexture.h"

namespace chrono {

/// Visual material.
class ChApi ChVisualMaterial {
  public:
    ChVisualMaterial();

    // Setting functions
    void SetAmbientColor(const ChVector<float>& rgb);
    void SetDiffuseColor(const ChVector<float>& rgb);
    void SetSpecularColor(const ChVector<float>& rgb);
    void SetEmissiveColor(const ChVector<float>& rgb);

    void SetSpecularExponent(float exponent);
    void SetOpacity(float o);
    void SetIllumination(int i);

    void SetKdTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);
    void SetKsTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);
    void SetNormalMapTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);
    void SetMetallicTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);
    void SetRoughnessTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);
    void SetOpacityTexture(const std::string& filename, float scale_x = 1, float scale_y = 1);

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
    float GetOpacity() const { return d; }
    int GetIllumination() const { return illum; }
    
    const std::string& GetKdTexture() const { return kd_texture.GetFilename(); }
    const std::string& GetKsTexture() const { return ks_texture.GetFilename(); }
    const std::string& GetNormalMapTexture() const { return normal_texture.GetFilename(); }
    const std::string& GetMetallicTexture() const { return metallic_texture.GetFilename(); }
    const std::string& GetRoughnessTexture() const { return roughness_texture.GetFilename(); }
    const std::string& GetOpacityTexture() const { return opacity_texture.GetFilename(); }

    const ChVector2<float>& GetKdTextureScale() const { return kd_texture.GetScale(); }
    const ChVector2<float>& GetKsTextureScale() const { return ks_texture.GetScale(); }
    const ChVector2<float>& GetNormalMapTextureScale() const { return normal_texture.GetScale(); }
    const ChVector2<float>& GetMetallicTextureScale() const { return metallic_texture.GetScale(); }
    const ChVector2<float>& GetRoughnessTextureScale() const { return roughness_texture.GetScale(); }
    const ChVector2<float>& GetOpacityTextureScale() const { return opacity_texture.GetScale(); }

    float GetFresnelExp() const { return fresnel_exp; }
    float GetFresnelMax() const { return fresnel_max; }
    float GetFresnelMin() const { return fresnel_min; }
    float GetRoughness() const { return roughness; }
    float GetMetallic() const { return metallic; }
    bool GetUseSpecularWorkflow() const { return use_specular_workflow; }
    unsigned short int GetClassID() const { return class_id; }
    unsigned short int GetInstanceID() const { return instance_id; }

    static std::shared_ptr<ChVisualMaterial> Default();

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

    ChTexture kd_texture;
    ChTexture ks_texture;
    ChTexture normal_texture;
    ChTexture metallic_texture;
    ChTexture roughness_texture;
    ChTexture opacity_texture;

    unsigned short int class_id;
    unsigned short int instance_id;
};

}  // end namespace chrono

#endif
