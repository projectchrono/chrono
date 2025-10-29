// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono_cascade/ChVisualShapeCascade.h"

namespace chrono {
namespace cascade {

ChVisualShapeCascade::ChVisualShapeCascade() {}

ChVisualShapeCascade::ChVisualShapeCascade(const TopoDS_Shape& ms) : mshape(ms) {}

ChVisualShapeCascade::~ChVisualShapeCascade() {}

void ChVisualShapeCascade::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChVisualShapeCascade>();
    // serialize parent class
    ChVisualShape::ArchiveOut(archive_out);
    // serialize all member data:
    // archive_out << ...; //***TODO*** serialize shape chunk using Cascade xml or STEP formats
}

/// Method to allow de serialization of transient data from archives.
void ChVisualShapeCascade::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChVisualShapeCascade>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(archive_in);
    // stream in all member data:
    // archive_in >> ...; //***TODO*** deserialize shape chunk using Cascade xml or STEP formats
}

}  // namespace cascade
}  // namespace chrono
