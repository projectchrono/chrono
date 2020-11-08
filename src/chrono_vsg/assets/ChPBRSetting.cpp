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

#include "chrono_vsg/assets/ChPBRSetting.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

namespace chrono {
namespace vsg3d {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPBRSetting)

ChPBRSetting::ChPBRSetting() {
    m_albedo.R = 0.5;
    m_albedo.G = 0.0;
    m_albedo.B = 0.0;
    m_albedo.A = 1.0;
    m_metallic = 1.0;
    m_roughness = 0.5;
    m_ao = 1.0;
}

ChPBRSetting::ChPBRSetting(float r, float g, float b, float metallic, float roughness, float ao) {
    m_albedo.R = r;
    m_albedo.G = g;
    m_albedo.B = b;
    m_albedo.A = 1.0;
    m_metallic = metallic;
    m_roughness = roughness;
    m_ao = ao;
}

ChPBRSetting::ChPBRSetting(ChColor& color, float metallic, float roughness, float ao) {
    m_albedo.R = color.R;
    m_albedo.G = color.G;
    m_albedo.B = color.B;
    m_albedo.A = color.A;
    m_metallic = metallic;
    m_roughness = roughness;
    m_ao = ao;
}

void ChPBRSetting::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChPBRSetting>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(m_albedo);
    marchive << CHNVP(m_metallic);
    marchive << CHNVP(m_roughness);
    marchive << CHNVP(m_ao);
}

void ChPBRSetting::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChPBRSetting>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_albedo);
    marchive >> CHNVP(m_metallic);
    marchive >> CHNVP(m_roughness);
    marchive >> CHNVP(m_ao);
}

}  // namespace vsg3d
}  // end namespace chrono
