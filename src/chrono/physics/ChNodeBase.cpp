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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/physics/ChNodeBase.h"

namespace chrono {

ChNodeBase::ChNodeBase() : offset_x(0), offset_w(0) {}

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

void ChNodeBase::NodeIntStateIncrement(const unsigned int off_x,
                                       ChState& x_new,
                                       const ChState& x,
                                       const unsigned int off_v,
                                       const ChStateDelta& Dv) {
    for (int i = 0; i < GetNdofX(); ++i) {
        x_new(off_x + i) = x(off_x + i) + Dv(off_v + i);
    }
}

void ChNodeBase::NodeIntStateGetIncrement(const unsigned int off_x,
                                          const ChState& x_new,
                                          const ChState& x,
                                          const unsigned int off_v,
                                          ChStateDelta& Dv) {
    for (int i = 0; i < GetNdofX(); ++i) {
        Dv(off_v + i) = x_new(off_x + i) - x(off_x + i);
    }
}

void ChNodeBase::ArchiveOut(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChNodeBase>();
    // serialize all member data:
}

void ChNodeBase::ArchiveIn(ChArchiveIn& marchive) {
    // version number
    /*int version =*/marchive.VersionRead<ChNodeBase>();
    // deserialize all member data:
}

}  // end namespace chrono
