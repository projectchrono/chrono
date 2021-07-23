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
#include "chrono/assets/ChCamera.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChCamera)

ChCamera::ChCamera()
    : position(ChVector<>(0, 1, 1)),
      aimpoint(VNULL),
      upvector(VECT_Y),
      angle(50),
      fov(3),
      hvratio(4.0 / 3),
      isometric(false){};

void ChCamera::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCamera>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(position);
    marchive << CHNVP(aimpoint);
    marchive << CHNVP(upvector);
    marchive << CHNVP(angle);
    marchive << CHNVP(fov);
    marchive << CHNVP(hvratio);
    marchive << CHNVP(isometric);
}

void ChCamera::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChCamera>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(position);
    marchive >> CHNVP(aimpoint);
    marchive >> CHNVP(upvector);
    marchive >> CHNVP(angle);
    marchive >> CHNVP(fov);
    marchive >> CHNVP(hvratio);
    marchive >> CHNVP(isometric);
}

}  // end namespace chrono
