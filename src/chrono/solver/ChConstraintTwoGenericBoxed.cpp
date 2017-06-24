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

#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChConstraintTwoGenericBoxed)

ChConstraintTwoGenericBoxed::ChConstraintTwoGenericBoxed(const ChConstraintTwoGenericBoxed& other)
    : ChConstraintTwoGeneric(other) {
    l_min = other.l_min;
    l_max = other.l_max;
}

double ChConstraintTwoGenericBoxed::Violation(double mc_i) {
    if ((l_i - 10e-5 < l_min) || (l_i + 10e-5 > l_max))
        return 0;
    return mc_i;
}

ChConstraintTwoGenericBoxed& ChConstraintTwoGenericBoxed::operator=(const ChConstraintTwoGenericBoxed& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintTwoGeneric::operator=(other);

    l_min = other.l_min;
    l_max = other.l_max;

    return *this;
}

void ChConstraintTwoGenericBoxed::Project() {
    if (l_i < l_min)
        l_i = l_min;
    if (l_i > l_max)
        l_i = l_max;
}

void ChConstraintTwoGenericBoxed::SetBoxedMinMax(double mmin, double mmax) {
    assert(mmin <= mmax);
    l_min = mmin;
    l_max = mmax;
}

void ChConstraintTwoGenericBoxed::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // serialize parent class too
    ChConstraintTwoGeneric::StreamOUT(mstream);

    // stream out all member data
    mstream << l_min;
    mstream << l_max;
}

void ChConstraintTwoGenericBoxed::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChConstraintTwoGeneric::StreamIN(mstream);

    // stream in all member data
    mstream >> l_min;
    mstream >> l_max;
}

}  // end namespace chrono
