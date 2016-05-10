//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#include "chrono/solver/ChConstraintTwoGenericBoxed.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChConstraintTwoGenericBoxed> a_registration_ChConstraintTwoGenericBoxed;

double ChConstraintTwoGenericBoxed::Violation(double mc_i) {
    if ((l_i - 10e-5 < l_min) || (l_i + 10e-5 > l_max))
        return 0;
    return mc_i;
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
