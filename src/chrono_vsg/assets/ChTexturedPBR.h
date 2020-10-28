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
// Authors: Rainer Gericke
// =============================================================================

#ifndef CH_TEXTURED_PBR_H
#define CH_TEXTURED_PBR_H

#include "chrono/assets/ChAsset.h"
#include "chrono_vsg/core/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

/// Base class for assets that define basic textures. Assets can be attached to ChBody objects.
/// Different post processing modules can handle textures in proper ways (ex for ray tracing, or
/// openGL visualization), or may also create specialized classes of textures with more properties.
class CH_VSG_API ChTexturedPBR : public ChAsset {
  protected:
    std::string materialPath;
    std::string materialName;

    std::string albedoFilename;
    std::string normalFilename;
    std::string metallicFilename;
    std::string roughnessFilename;
    std::string aoFilename;

  public:
    ChTexturedPBR() {
        materialPath = "vsg/textures/pbr";
        materialName = "";
    }
    ChTexturedPBR(const char* theMaterialName);
    ChTexturedPBR(const std::string& theMaterialName);

    virtual ~ChTexturedPBR() {}

    void SetAlbedoTextureFilename(const std::string& albedo_name) { albedoFilename = albedo_name; }
    void SetNormalTextureFilename(const std::string& normal_name) { normalFilename = normal_name; }
    void SetMetallicTextureFilename(const std::string& metallic_name) { metallicFilename = metallic_name; }
    void SetRoughnessTextureFilename(const std::string& roughness_name) { roughnessFilename = roughness_name; }
    void SetAOTextureFilename(const std::string& ao_name) { aoFilename = ao_name; }
    const std::string& GetAlbedoTextureFilename() const { return albedoFilename; }
    const std::string& GetNormalTextureFilename() const { return normalFilename; }
    const std::string& GetMetallicTextureFilename() const { return metallicFilename; }
    const std::string& GetRoughnessTextureFilename() const { return roughnessFilename; }
    const std::string& GetAOTextureFilename() const { return aoFilename; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // namespace vsg3d
CH_CLASS_VERSION(vsg3d::ChTexturedPBR, 0)
}  // end namespace chrono

#endif
