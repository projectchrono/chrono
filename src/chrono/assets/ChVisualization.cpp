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

#include "chrono/assets/ChVisualization.h"

namespace chrono {

ChVisualization::ChVisualization() : Pos(0), Rot(1), visible(true), is_static(false), fading(0) {}

void ChVisualization::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualization>();
    // serialize parent class
    ChAsset::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(Pos);
    marchive << CHNVP(Rot);
    marchive << CHNVP(visible);
    marchive << CHNVP(color);
    marchive << CHNVP(fading);
}

void ChVisualization::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChVisualization>();
    // deserialize parent class
    ChAsset::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(Pos);
    marchive >> CHNVP(Rot);
    marchive >> CHNVP(visible);
    marchive >> CHNVP(color);
    marchive >> CHNVP(fading);
}

}  // namespace chrono
