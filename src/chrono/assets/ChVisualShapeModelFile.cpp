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

#include "chrono/assets/ChVisualShapeModelFile.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChVisualShapeModelFile)

ChVisualShapeModelFile::ChVisualShapeModelFile() : filename(""), scale(1) {
    SetMutable(false);
}

ChVisualShapeModelFile::ChVisualShapeModelFile(const std::string& fname) : filename(fname), scale(1) {
    SetMutable(false);
}

void ChVisualShapeModelFile::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChVisualShapeModelFile>();
    // serialize parent class
    ChVisualShape::ArchiveOut(marchive);
    // serialize all member data:
    marchive << CHNVP(filename);
    marchive << CHNVP(scale);
}

void ChVisualShapeModelFile::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChVisualShapeModelFile>();
    // deserialize parent class
    ChVisualShape::ArchiveIn(marchive);
    // stream in all member data:
    marchive >> CHNVP(filename);
    marchive >> CHNVP(scale);
}

}  // end namespace chrono
