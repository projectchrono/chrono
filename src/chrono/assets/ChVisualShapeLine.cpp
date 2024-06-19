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

#include "chrono/assets/ChVisualShapeLine.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeLine)

ChVisualShapeLine::ChVisualShapeLine() : npoints(200), thickness(10.0) {
    gline = chrono_types::make_shared<ChLineSegment>();
}

ChVisualShapeLine::ChVisualShapeLine(std::shared_ptr<ChLine>& mline) : npoints(200), thickness(10.0), gline(mline) {}

void ChVisualShapeLine::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeLine>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    archive_out << CHNVP(gline);
    archive_out << CHNVP(npoints);
    archive_out << CHNVP(thickness);
}

void ChVisualShapeLine::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeLine>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    archive_in >> CHNVP(gline);
    archive_in >> CHNVP(npoints);
    archive_in >> CHNVP(thickness);
}

}  // end namespace chrono
