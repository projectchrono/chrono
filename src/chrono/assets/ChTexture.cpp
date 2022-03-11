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
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/assets/ChTexture.h"
#include "chrono/core/ChClassFactory.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChTexture)

ChTexture::ChTexture() : m_filename(""), scale_x(1), scale_y(1), m_scale(ChVector2<float>(1, 1)) {}

ChTexture::ChTexture(const char* filename)
    : m_filename(filename), scale_x(1), scale_y(1), m_scale(ChVector2<float>(1, 1)) {}

ChTexture::ChTexture(const std::string& filename, ChVector2<float> scale)
    : scale_x(scale.x()), scale_y(scale.y()), m_scale(scale), m_filename(filename) {}

void ChTexture::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTexture>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(m_filename);
}

void ChTexture::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChTexture>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(m_filename);
}

}  // end namespace chrono
