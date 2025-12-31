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

#include "chrono/solver/ChConstraintContactTangential.h"

namespace chrono {

ChConstraintContactTangential::ChConstraintContactTangential() {
    mode = ChConstraint::Mode::FRICTION;
}

ChConstraintContactTangential::ChConstraintContactTangential(const ChConstraintContactTangential& other)
    : ChConstraintTwoTuples(other) {
}

ChConstraintContactTangential& ChConstraintContactTangential::operator=(const ChConstraintContactTangential& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintTwoTuples::operator=(other);

    return *this;
}

}  // end namespace chrono
