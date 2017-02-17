// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChNodeBase.h"

namespace chrono {

ChNodeBase::ChNodeBase() {
    offset_x = 0;
    offset_w = 0;
}

ChNodeBase::ChNodeBase(const ChNodeBase& other) {
    offset_x = other.offset_x;
    offset_w = other.offset_w;
}

ChNodeBase& ChNodeBase::operator=(const ChNodeBase& other) {
    if (&other == this)
        return *this;

    offset_x = other.offset_x;
    offset_w = other.offset_w;

    return *this;
}

void ChNodeBase::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeBase>();
    // serialize all member data:
}

/// Method to allow de serialization of transient data from archives.
void ChNodeBase::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChNodeBase>();
    // deserialize all member data:
}

}  // end namespace chrono
