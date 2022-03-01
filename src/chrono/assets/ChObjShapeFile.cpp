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

#include "chrono/assets/ChObjShapeFile.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChObjShapeFile)

ChObjShapeFile::ChObjShapeFile() : filename("") {
    SetMutable(false);
}

ChObjShapeFile::ChObjShapeFile(const std::string& fname) : filename(fname) {
    SetMutable(false);
}

void ChObjShapeFile::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChObjShapeFile>();
    // serialize parent class
    ChVisualShape::ArchiveOUT(marchive);
    // serialize all member data:
    marchive << CHNVP(filename);
}

void ChObjShapeFile::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChObjShapeFile>();
    // deserialize parent class
    ChVisualShape::ArchiveIN(marchive);
    // stream in all member data:
    marchive >> CHNVP(filename);
}

}  // end namespace chrono
