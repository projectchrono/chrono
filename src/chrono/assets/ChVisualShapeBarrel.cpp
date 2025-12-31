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

#include "chrono/assets/ChVisualShapeBarrel.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeBarrel)

ChVisualShapeBarrel::ChVisualShapeBarrel() {
    SetMutable(false);
}

ChVisualShapeBarrel::ChVisualShapeBarrel(double Y_low,
                                         double Y_high,
                                         double axis_vert,
                                         double axis_hor,
                                         double R_offset)
    : Hlow(Y_low), Hsup(Y_high), Rvert(axis_vert / 2), Rhor(axis_hor / 2), Roffset(R_offset) {
    SetMutable(false);
}

ChAABB ChVisualShapeBarrel::GetBoundingBox() const {
    double y_min = std::min(Hlow, -Rvert);
    double y_max = std::max(Hsup, +Rvert);
    return ChAABB(ChVector3d(-Rhor, y_min, -Rhor), ChVector3d(+Rhor, y_max, +Rhor));
}

void ChVisualShapeBarrel::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeBarrel>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(Hlow);
    archive_out << CHNVP(Hsup);
    archive_out << CHNVP(Rvert);
    archive_out << CHNVP(Rhor);
    archive_out << CHNVP(Roffset);
}

/// Method to allow de serialization of transient data from archives.
void ChVisualShapeBarrel::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeBarrel>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(Hlow);
    archive_in >> CHNVP(Hsup);
    archive_in >> CHNVP(Rvert);
    archive_in >> CHNVP(Rhor);
    archive_in >> CHNVP(Roffset);
}

}  // end namespace chrono
