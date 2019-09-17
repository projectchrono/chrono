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

#include "chrono/assets/ChPathShape.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChPathShape)

ChPathShape::ChPathShape() : npoints(200) {
    gpath = chrono_types::make_shared<geometry::ChLinePath>();
}

ChPathShape::ChPathShape(std::shared_ptr<geometry::ChLinePath>& mpath) : npoints(200), gpath(mpath) {}

void ChPathShape::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChPathShape>();
    // serialize parent class
    ChVisualization::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(gpath);
}

void ChPathShape::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChPathShape>();
    // deserialize parent class
    ChVisualization::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(gpath);
}

}  // end namespace chrono
