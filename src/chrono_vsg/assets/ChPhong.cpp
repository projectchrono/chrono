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

#include "chrono_vsg/assets/ChPhong.h"
#include "chrono/core/ChClassFactory.h"

namespace chrono {
namespace vsg3d {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPhong)

ChPhong::ChPhong() {
    ambientColor = vsg::vec3(0.0215, 0.1745, 0.0215);
    diffuseColor = vsg::vec3(0.07568, 0.61424, 0.07568);
    specularColor = vsg::vec3(0.633, 0.727811, 0.633);
    shininess = 0.6 * 128.0;
}

ChPhong::ChPhong(ChVSGPhongMaterial& material) {
    ambientColor = material.ambientColor;
    diffuseColor = material.diffuseColor;
    specularColor = material.specularColor;
    shininess = material.shininess;
}

void ChPhong::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChPhong>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(ambientColor[0]);
    marchive << CHNVP(ambientColor[1]);
    marchive << CHNVP(ambientColor[2]);
    marchive << CHNVP(diffuseColor[0]);
    marchive << CHNVP(diffuseColor[1]);
    marchive << CHNVP(diffuseColor[2]);
    marchive << CHNVP(specularColor[0]);
    marchive << CHNVP(specularColor[1]);
    marchive << CHNVP(specularColor[2]);
    marchive << CHNVP(shininess);
}

void ChPhong::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChPhong>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(ambientColor[0]);
    marchive >> CHNVP(ambientColor[1]);
    marchive >> CHNVP(ambientColor[2]);
    marchive >> CHNVP(diffuseColor[0]);
    marchive >> CHNVP(diffuseColor[1]);
    marchive >> CHNVP(diffuseColor[2]);
    marchive >> CHNVP(specularColor[0]);
    marchive >> CHNVP(specularColor[1]);
    marchive >> CHNVP(specularColor[2]);
    marchive >> CHNVP(shininess);
}

}  // namespace vsg3d
}  // end namespace chrono
