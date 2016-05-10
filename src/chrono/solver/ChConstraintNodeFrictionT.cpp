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

#include "chrono/solver/ChConstraintNodeFrictionT.h"

namespace chrono {

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChConstraintNodeFrictionT> a_registration_ChConstraintNodeFrictionT;

double ChConstraintNodeFrictionT::Violation(double mc_i) {
    return 0.0;  //***TO DO*** compute true violation when in sticking?
}

void ChConstraintNodeFrictionT::StreamOUT(ChStreamOutBinary& mstream) {
    // class version number
    mstream.VersionWrite(1);

    // serialize parent class too
    ChConstraintTwoGeneric::StreamOUT(mstream);
}

void ChConstraintNodeFrictionT::StreamIN(ChStreamInBinary& mstream) {
    // class version number
    int version = mstream.VersionRead();

    // deserialize parent class too
    ChConstraintTwoGeneric::StreamIN(mstream);
}

}  // end namespace chrono
