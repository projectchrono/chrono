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

#ifndef CH_PBR_MAPS_H
#define CH_PBR_MAPS_H

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono_vsg/core/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

/// Base class for assets that define basic textures. Assets can be attached to ChBody objects.
/// Different post processing modules can handle textures in proper ways (ex for ray tracing, or
/// openGL visualization), or may also create specialized classes of textures with more properties.
class CH_VSG_API ChPBRMaps : public ChAsset {
  public:
    ChPBRMaps();
    ChPBRMaps(std::string mapDir);
    virtual ~ChPBRMaps() {}
    std::string& GetMaterialName() { return m_matName; }
    std::string& GetAlbedoMapPath() { return m_albedoPath; }
    std::string& GetNormalMapPath() { return m_normalPath; }
    std::string& GetMetallicMapPath() { return m_metallicPath; }
    std::string& GetRoughnessMapPath() { return m_roughnessPath; }
    std::string& GetAoMapPath() { return m_aoPath; }

    void SetAlbedoMapPath(std::string& albedo) { m_albedoPath = albedo; }
    void SetNormalMapPath(std::string& normal) { m_normalPath = normal; }
    void SetMetallicMapPath(std::string& metallic) { m_metallicPath = metallic; }
    void SetRoughnessMapPath(std::string& roughness) { m_roughnessPath = roughness; }
    void SetAoMapPath(std::string& ao) { m_aoPath = ao; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  protected:
    std::string m_matName;
    std::string m_albedoPath;
    std::string m_normalPath;
    std::string m_metallicPath;
    std::string m_roughnessPath;
    std::string m_aoPath;
};

}  // namespace vsg3d
CH_CLASS_VERSION(vsg3d::ChPBRMaps, 0)
}  // end namespace chrono

#endif
