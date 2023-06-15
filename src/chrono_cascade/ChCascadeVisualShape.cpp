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

#include "chrono_cascade/ChCascadeVisualShape.h"

#include <TopoDS_Shape.hxx>

namespace chrono {
namespace cascade {

ChCascadeVisualShape::ChCascadeVisualShape() {}

ChCascadeVisualShape::ChCascadeVisualShape(const TopoDS_Shape& ms) : mshape(ms) {}

ChCascadeVisualShape::~ChCascadeVisualShape() {}

void ChCascadeVisualShape::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChCascadeVisualShape>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    // marchive << ...; //***TODO*** serialize shape chunk using Cascade xml or STEP formats
}

/// Method to allow de serialization of transient data from archives.
void ChCascadeVisualShape::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChCascadeVisualShape>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    // marchive >> ...; //***TODO*** deserialize shape chunk using Cascade xml or STEP formats
}

}  // namespace cascade
}  // namespace chrono
