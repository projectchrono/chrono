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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/assets/ChCamera.h"
#include "chrono/utils/ChConstants.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChCamera)

ChCamera::ChCamera()
    : m_owner(nullptr),
      position(ChVector3d(0, 1, 1)),
      aimpoint(VNULL),
      upvector(VECT_Y),
      angle(50),
      fov(3),
      hvratio(CH_4_3),
      isometric(false){};

void ChCamera::ArchiveOut(ChArchiveOut& archive) {
    // version number
    archive.VersionWrite<ChCamera>();
    // serialize all member data:
    archive << CHNVP(position);
    archive << CHNVP(aimpoint);
    archive << CHNVP(upvector);
    archive << CHNVP(angle);
    archive << CHNVP(fov);
    archive << CHNVP(hvratio);
    archive << CHNVP(isometric);
}

void ChCamera::ArchiveIn(ChArchiveIn& archive) {
    // version number
    /*int version =*/archive.VersionRead<ChCamera>();
    // stream in all member data:
    archive >> CHNVP(position);
    archive >> CHNVP(aimpoint);
    archive >> CHNVP(upvector);
    archive >> CHNVP(angle);
    archive >> CHNVP(fov);
    archive >> CHNVP(hvratio);
    archive >> CHNVP(isometric);
}

}  // end namespace chrono
