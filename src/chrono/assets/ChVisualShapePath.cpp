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

#include "chrono/assets/ChVisualShapePath.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapePath)

ChVisualShapePath::ChVisualShapePath() : npoints(200), thickness(10.0) {
    gpath = chrono_types::make_shared<geometry::ChLinePath>();
}

ChVisualShapePath::ChVisualShapePath(std::shared_ptr<geometry::ChLinePath>& mpath) : npoints(200), thickness(10.0), gpath(mpath) {}

void ChVisualShapePath::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapePath>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(gpath);
    marchive << CHNVP(npoints);
    marchive << CHNVP(thickness);
}

void ChVisualShapePath::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShapePath>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(gpath);
    marchive >> CHNVP(npoints);
    marchive >> CHNVP(thickness);
}

}  // end namespace chrono
