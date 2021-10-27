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
// Authors: Asher Elmquist
// =============================================================================
//
// Visual assets that can be used for higher quality rendering such as that
// used by the sensing module. These materials follow, in part, from the
// wavefront obj mtl specification.
//
// =============================================================================

#ifndef CHVISUALMATERIAL_H
#define CHVISUALMATERIAL_H

#include "chrono/assets/ChAsset.h"

namespace chrono {

/// Class for setting a color (used by ChVisualization)
class ChApi ChVisualMaterial : public ChAsset {
  public:
    /// Constructors
    ChVisualMaterial();

    // Setting functions
    void SetAmbientColor(ChVector<float> rgb);
    void SetDiffuseColor(ChVector<float> rgb);
    void SetSpecularColor(ChVector<float> rgb);
    void SetSpecularExponent(float exponent);
    void SetTransparency(float tr);
    void SetKdTexture(std::string filename) { kd_texture = filename; };
    void SetKsTexture(std::string filename) { ks_texture = filename; };
    void SetNormalMapTexture(std::string filename) { normal_texture = filename; };
    void SetMetallicTexture(std::string filename) { metallic_texture = filename; };
    void SetRoughnessTexture(std::string filename) { roughness_texture = filename; };
    void SetOpacityTexture(std::string filename) { opacity_texture = filename; };
    void SetFresnelExp(float exp);
    void SetFresnelMax(float max);
    void SetFresnelMin(float min);
    void SetRoughness(float r);
    void SetMetallic(float m);
    void SetUseSpecularWorkflow(bool s) { use_specular_workflow = s; }
    void SetClassID(unsigned short int id) { class_id = id; }
    void SetInstanceID(unsigned short int id) { instance_id = id; }

    // accessor functions
    ChVector<float> GetAmbientColor() { return Ka; }
    ChVector<float> GetDiffuseColor() { return Kd; }
    ChVector<float> GetSpecularColor() { return Ks; }
    float GetSpecularExponent() { return Ns; }
    float GetTransparency() { return d; }
    std::string GetKdTexture() { return kd_texture; };
    std::string GetKsTexture() { return ks_texture; };
    std::string GetNormalMapTexture() { return normal_texture; };
    std::string GetMetallicTexture() { return metallic_texture; };
    std::string GetRoughnessTexture() { return roughness_texture; };
    std::string GetOpacityTexture() { return opacity_texture; };
    float GetFresnelExp() { return fresnel_exp; }
    float GetFresnelMax() { return fresnel_max; }
    float GetFresnelMin() { return fresnel_min; }
    float GetRoughness() { return roughness; }
    float GetMetallic() { return metallic; }
    bool GetUseSpecularWorkflow() { return use_specular_workflow; }
    unsigned short int GetClassID() { return class_id; }
    unsigned short int GetInstanceID() { return instance_id; }

  private:
    ChVector<float> Ka;  // ambient color 0-1
    ChVector<float> Kd;  // diffuse color   0-1
    ChVector<float> Ks;  // specular color 0-1

    float fresnel_max;
    float fresnel_min;
    float fresnel_exp;
    float Ns;  // specular exponent
    float d;   // transparency

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
