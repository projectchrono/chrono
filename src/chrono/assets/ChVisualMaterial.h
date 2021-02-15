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

    /// Setting function
    void SetAmbientColor(ChVector<float> rgb);
    void SetDiffuseColor(ChVector<float> rgb);
    void SetSpecularColor(ChVector<float> rgb);
    void SetSpecularExponent(float exponent);
    void SetTransparency(float tr);
    void SetKdTexture(std::string filename) { kd_texture = filename; };
    void SetNormalMapTexture(std::string filename){ normal_texture = filename; };
    void SetFresnelExp(float exp);
    void SetFresnelMax(float max);
    void SetFresnelMin(float min);
    void SetRoughness(float r);

    // accessor functions
    ChVector<float> GetAmbientColor() { return Ka; }
    ChVector<float> GetDiffuseColor() { return Kd; }
    ChVector<float> GetSpecularColor() { return Ks; }
    float GetSpecularExponent() { return Ns; }
    float GetTransparency() { return d; }
    std::string GetKdTexture() { return kd_texture; };
    std::string GetNormalMapTexture(){ return normal_texture; };
    float GetFresnelExp() { return fresnel_exp; }
    float GetFresnelMax() { return fresnel_max; }
    float GetFresnelMin() { return fresnel_min; }
    float GetRoughness() { return roughness; }

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

    bool has_texture;
    std::string kd_texture;
    std::string normal_texture;
};

}  // end namespace chrono

#endif
