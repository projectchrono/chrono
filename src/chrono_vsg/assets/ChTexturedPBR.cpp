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

#include "chrono_vsg/assets/ChTexturedPBR.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

namespace chrono {
namespace vsg3d {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTexturedPBR)

ChTexturedPBR::ChTexturedPBR(const char* theMaterialName) {
    filesystem::path materialPath(theMaterialName);

    materialName = materialPath.filename();

    std::string thePath(theMaterialName);

    albedoFilename = thePath + "/" + materialName + "_albedo.png";
    normalFilename = thePath + "/" + materialName + "_normal.png";
    metallicFilename = thePath + "/" + materialName + "_metallic.png";
    roughnessFilename = thePath + "/" + materialName + "_roughness.png";
    aoFilename = thePath + "/" + materialName + "_ao.png";
}

ChTexturedPBR::ChTexturedPBR(const std::string& theMaterialName) {
    filesystem::path materialPath(theMaterialName);

    materialName = materialPath.filename();
    std::string thePath(theMaterialName);

    albedoFilename = thePath + "/" + materialName + "_albedo.png";
    normalFilename = thePath + "/" + materialName + "_normal.png";
    metallicFilename = thePath + "/" + materialName + "_metallic.png";
    roughnessFilename = thePath + "/" + materialName + "_roughness.png";
    aoFilename = thePath + "/" + materialName + "_ao.png";
}

void ChTexturedPBR::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTexturedPBR>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(materialName);
}

void ChTexturedPBR::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChTexturedPBR>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(materialName);
}

}  // namespace vsg3d
}  // end namespace chrono
