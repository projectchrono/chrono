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

#include "chrono_vsg/assets/ChPBRMaps.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

namespace chrono {
namespace vsg3d {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPBRMaps)

ChPBRMaps::ChPBRMaps() {}

ChPBRMaps::ChPBRMaps(std::string mapDir) {
    filesystem::path root(mapDir);
    if (root.exists()) {
        m_matName = root.filename();
        m_albedoPath = mapDir + "/" + m_matName + "_albedo.png";
        m_normalPath = mapDir + "/" + m_matName + "_normal.png";
        m_metallicPath = mapDir + "/" + m_matName + "_metallic.png";
        m_roughnessPath = mapDir + "/" + m_matName + "_roughness.png";
        m_aoPath = mapDir + "/" + m_matName + "_ao.png";
    } else {
        GetLog() << "ChPBRMaps: selected material does not exist!\n";
    }
}

void ChPBRMaps::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChPBRMaps>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(m_matName);
    marchive << CHNVP(m_albedoPath);
    marchive << CHNVP(m_normalPath);
    marchive << CHNVP(m_metallicPath);
    marchive << CHNVP(m_roughnessPath);
    marchive << CHNVP(m_aoPath);
}

void ChPBRMaps::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChPBRMaps>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_matName);
    marchive >> CHNVP(m_albedoPath);
    marchive >> CHNVP(m_normalPath);
    marchive >> CHNVP(m_metallicPath);
    marchive >> CHNVP(m_roughnessPath);
    marchive >> CHNVP(m_aoPath);
}

}  // namespace vsg3d
}  // end namespace chrono
