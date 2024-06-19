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

ChTexture::ChTexture() : m_filename(""), m_scale(ChVector2f(1, 1)) {}

ChTexture::ChTexture(const char* filename) : m_filename(filename), m_scale(ChVector2f(1, 1)) {}

ChTexture::ChTexture(const std::string& filename, float scale_x, float scale_y)
    : m_filename(filename), m_scale(ChVector2f(scale_x, scale_y)) {}

void ChTexture::SetScale(float sx, float sy) {
    m_scale.x() = sx;
    m_scale.y() = sy;
}

void ChTexture::SetScale(const ChVector2f& scale) {
    m_scale = scale;
}

void ChTexture::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChTexture>();
    // serialize all member data:
    archive_out << CHNVP(m_filename);
    archive_out << CHNVP(m_scale);
}

void ChTexture::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChTexture>();
    // stream in all member data:
    archive_in >> CHNVP(m_filename);
    archive_in >> CHNVP(m_scale);
}

}  // end namespace chrono
