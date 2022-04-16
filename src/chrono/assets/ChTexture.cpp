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

ChTexture::ChTexture() : m_filename(""), m_scale(ChVector2<float>(1, 1)) {}

ChTexture::ChTexture(const char* filename) : m_filename(filename), m_scale(ChVector2<float>(1, 1)) {}

ChTexture::ChTexture(const std::string& filename, float scale_x, float scale_y)
    : m_filename(filename), m_scale(ChVector2<float>(scale_x, scale_y)) {}

void ChTexture::SetScale(float sx, float sy) {
    m_scale.x() = sx;
    m_scale.y() = sy;
}

void ChTexture::SetScale(const ChVector2<float>& scale) {
    m_scale = scale;
}

void ChTexture::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChTexture>();
    // serialize all member data:
    marchive << CHNVP(m_filename);
}

void ChTexture::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChTexture>();
    // stream in all member data:
    marchive >> CHNVP(m_filename);
}

}  // end namespace chrono
