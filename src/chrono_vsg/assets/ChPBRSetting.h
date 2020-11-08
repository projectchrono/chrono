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

#ifndef CH_PBR_SETTING_H
#define CH_PBR_SETTING_H

#include "chrono/assets/ChAsset.h"
#include "chrono/assets/ChColor.h"
#include "chrono_vsg/core/ChApiVSG.h"

namespace chrono {
namespace vsg3d {

/// Base class for assets that define basic textures. Assets can be attached to ChBody objects.
/// Different post processing modules can handle textures in proper ways (ex for ray tracing, or
/// openGL visualization), or may also create specialized classes of textures with more properties.
class CH_VSG_API ChPBRSetting : public ChAsset {
  protected:
    ChColor m_albedo;
    float m_metallic;
    float m_roughness;
    float m_ao;

  public:
    ChPBRSetting();
    ChPBRSetting(float r, float g, float b, float metallic, float roughness, float ao);
    ChPBRSetting(ChColor& color, float metallic = 1.0, float roughness = 0.5, float ao = 1.0);

    virtual ~ChPBRSetting() {}

    ChColor& GetAlbedo() { return m_albedo; }
    float GetMetallic() { return m_metallic; }
    float GetRoughness() { return m_roughness; }
    float GetAO() { return m_ao; }

    void SetAlbedo(ChColor& albedo) {
        m_albedo.R = albedo.R;
        m_albedo.G = albedo.G;
        m_albedo.B = albedo.B;
        m_albedo.A = albedo.A;
    }
    void SetMetallic(float metallic) { m_metallic = metallic; }
    void SetRoughness(float roughness) { m_roughness = roughness; }
    void SetAO(float ao) { m_ao = ao; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

}  // namespace vsg3d
CH_CLASS_VERSION(vsg3d::ChPBRSetting, 0)
}  // end namespace chrono

#endif
