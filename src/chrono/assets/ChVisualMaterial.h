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
#include "chrono/assets/ChColor.h"

namespace chrono {

/// Definition of a visual material.
class ChApi ChVisualMaterial {
  public:
    ChVisualMaterial();

    // Setting functions
    void SetAmbientColor(const ChColor& rgb);
    void SetDiffuseColor(const ChColor& rgb);
    void SetSpecularColor(const ChColor& rgb);
    void SetEmissiveColor(const ChColor& rgb);

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
    const ChColor& GetAmbientColor() const { return Ka; }
    const ChColor& GetDiffuseColor() const { return Kd; }
    const ChColor& GetSpecularColor() const { return Ks; }
    const ChColor& GetEmissiveColor() const { return Ke; }
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
    ChColor Ka;  ///< ambient color
    ChColor Kd;  ///< diffuse color
    ChColor Ks;  ///< specular color
    ChColor Ke;  ///< emissive color

    float fresnel_max;
    float fresnel_min;
    float fresnel_exp;
    float Ns;  ///< specular exponent
    float d;   ///< opacity

    int illum;  ///< illumination model (see http://www.fileformat.info/format/material/)

    float roughness;
    float metallic;

    bool use_specular_workflow;

    ChTexture kd_texture;         ///< diffuse texture map
    ChTexture ks_texture;         ///< specular texture map
    ChTexture normal_texture;     ///< normal texture map
    ChTexture metallic_texture;   ///< metallic texture map
    ChTexture roughness_texture;  ///< roughness texture map
    ChTexture opacity_texture;    ///< opacity texture map

    unsigned short int class_id;
    unsigned short int instance_id;
};

}  // end namespace chrono

#endif
