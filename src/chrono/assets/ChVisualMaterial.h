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

#include "chrono/core/ChVector3.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColor.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

/// Definition of a visual material.
class ChApi ChVisualMaterial {
  public:
    ChVisualMaterial();

    // Setting functions

    void SetAmbientColor(const ChColor& rgb);
    void SetDiffuseColor(const ChColor& rgb);
    void SetSpecularColor(const ChColor& rgb);
    void SetEmissiveColor(const ChColor& rgb);
    void SetEmissivePower(const float& power);

    void SetSpecularExponent(float exponent);
    void SetOpacity(float o);
    void SetIllumination(int i);

    void SetKdTexture(const std::string& filename);
    void SetKsTexture(const std::string& filename);
    void SetKeTexture(const std::string& filename);
    void SetNormalMapTexture(const std::string& filename);
    void SetMetallicTexture(const std::string& filename);
    void SetRoughnessTexture(const std::string& filename);
    void SetOpacityTexture(const std::string& filename);
    void SetWeightTexture(const std::string& filename);
    void SetDisplacementTexture(const std::string& filename);
    void SetAmbientOcclusionTexture(const std::string& filename);

    /// Apply the specified texture scaling to all textures in this material.
    void SetTextureScale(float scale_x, float scale_y);

    void SetFresnelExp(float exp);
    void SetFresnelMax(float max);
    void SetFresnelMin(float min);
    void SetRoughness(float r);
    void SetMetallic(float m);
    void SetAnisotropy(float a);
    void SetUseSpecularWorkflow(bool s) { use_specular_workflow = s; }
    /// @brief Enable or disable the use of the Hapke material model. We implement the modern hapke model descried in  https://doi.org/10.1002/2013JE004580
    void SetUseHapke(bool h) {use_hapke = h;}

    void SetClassID(unsigned short int id) { class_id = id; }
    void SetInstanceID(unsigned short int id) { instance_id = id; }

    /// @brief  Set the Hapke material parameters. Note that in our implementation, we ignore the impact of coherent backscatter. 
    /// @param w  single scattering albedo
    /// @param b  shape controlling parameter for the amplitude of backward and forward scatter of particles
    /// @param c  weighting factor that controls the contribution of backward and forward scatter.
    /// @param B_s0 Amplitude of the opposition effect caused by shadow hiding
    /// @param h_s  Angular width of the opposition effect caused by shadow hiding.
    /// @param phi Fillig factor of the regolith
    /// @param theta_p Effective value of the photometric roughness which controls the surface roughness
    void SetHapkeParameters(float w, float b, float c, float B_s0, float h_s, float phi, float theta_p);

    // Accessor functions

    const ChColor& GetAmbientColor() const { return Ka; }
    const ChColor& GetDiffuseColor() const { return Kd; }
    const ChColor& GetSpecularColor() const { return Ks; }
    const ChColor& GetEmissiveColor() const { return Ke; }
    const float& GetEmissivePower() const {return emissive_power;}
    float GetSpecularExponent() const { return Ns; }
    float GetOpacity() const { return d; }
    int GetIllumination() const { return illum; }

    const std::string& GetKdTexture() const { return kd_texture.GetFilename(); }
    const std::string& GetKsTexture() const { return ks_texture.GetFilename(); }
    const std::string& GetKeTexture() const { return ke_texture.GetFilename(); }
    const std::string& GetNormalMapTexture() const { return normal_texture.GetFilename(); }
    const std::string& GetMetallicTexture() const { return metallic_texture.GetFilename(); }
    const std::string& GetRoughnessTexture() const { return roughness_texture.GetFilename(); }
    const std::string& GetOpacityTexture() const { return opacity_texture.GetFilename(); }
    const std::string& GetWeightTexture() const { return weight_texture.GetFilename(); }
    const std::string& GetDisplacementTexture() const { return disp_texture.GetFilename(); }
    const std::string& GetAmbientOcclusionTexture() const { return ao_texture.GetFilename(); }

    const ChVector2f& GetTextureScale() const;

    float GetFresnelExp() const { return fresnel_exp; }
    float GetFresnelMax() const { return fresnel_max; }
    float GetFresnelMin() const { return fresnel_min; }
    float GetRoughness() const { return roughness; }
    float GetMetallic() const { return metallic; }
    float GetAnisotropy() const {return anisotropy; }
    bool GetUseSpecularWorkflow() const { return use_specular_workflow; }
    bool GetUseHapke() const {return use_hapke;}
    float GetHapkeW() const {return hapke_w;}
    float GetHapkeB() const {return hapke_b;}
    float GetHapkeC() const {return hapke_c;}
    float GetHapkeBs0() const {return hapke_B_s0;}
    float GetHapkeHs() const {return hapke_h_s;}
    float GetHapkePhi() const {return hapke_phi;}
    float GetHapkeRoughness() const {return hapke_theta_p;}

    unsigned short int GetClassID() const { return class_id; }
    unsigned short int GetInstanceID() const { return instance_id; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    /// Create a default material.
    static std::shared_ptr<ChVisualMaterial> Default();

  private:
    ChColor Ka;  ///< ambient color
    ChColor Kd;  ///< diffuse color
    ChColor Ks;  ///< specular color
    ChColor Ke;  ///< emissive color

    float emissive_power;

    float fresnel_max;
    float fresnel_min;
    float fresnel_exp;
    float Ns;  ///< specular exponent
    float d;   ///< opacity

    int illum;  ///< illumination model (see http://www.fileformat.info/format/material/)

    float roughness;
    float metallic;
    float anisotropy;

    bool use_specular_workflow;
    bool use_hapke;

    ChTexture kd_texture;         ///< diffuse texture map
    ChTexture ks_texture;         ///< specular texture map
    ChTexture ke_texture;         ///< emissive texture map
    ChTexture normal_texture;     ///< normal texture map
    ChTexture metallic_texture;   ///< metallic texture map
    ChTexture roughness_texture;  ///< roughness texture map
    ChTexture opacity_texture;    ///< opacity texture map
    ChTexture weight_texture;     ///< weight texture map
    ChTexture disp_texture;       ///< displacement map
    ChTexture ao_texture;         ///< ambient occlusion map

    unsigned short int class_id;
    unsigned short int instance_id;


    // Hapke material parameters
    float hapke_w; // single scattering albedo
    float hapke_b; // shape controlling parameter for the amplitude of backward and forward scatter of particles
    float hapke_c; // weighting factor that controls the contribution of backward and forward scatter.
    float hapke_B_s0;
    float hapke_h_s;
    float hapke_phi;
    float hapke_theta_p;
    
};

typedef std::shared_ptr<ChVisualMaterial> ChVisualMaterialSharedPtr;

/// @} chrono_assets

}  // end namespace chrono

#endif
