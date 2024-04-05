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

#include "chrono/assets/ChVisualShapeSurface.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeSurface)

void ChVisualShapeSurface::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeSurface>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gsurface);
    archive_out << CHNVP(wireframe);
    archive_out << CHNVP(resolution_U);
    archive_out << CHNVP(resolution_V);
}

void ChVisualShapeSurface::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeSurface>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gsurface);
    archive_in >> CHNVP(wireframe);
    archive_in >> CHNVP(resolution_U);
    archive_in >> CHNVP(resolution_V);
}

}  // end namespace chrono
