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
    gpath = chrono_types::make_shared<ChLinePath>();
}

ChVisualShapePath::ChVisualShapePath(std::shared_ptr<ChLinePath>& mpath)
    : npoints(200), thickness(10.0), gpath(mpath) {}

void ChVisualShapePath::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapePath>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gpath);
    archive_out << CHNVP(npoints);
    archive_out << CHNVP(thickness);
}

void ChVisualShapePath::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapePath>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gpath);
    archive_in >> CHNVP(npoints);
    archive_in >> CHNVP(thickness);
}

}  // end namespace chrono
