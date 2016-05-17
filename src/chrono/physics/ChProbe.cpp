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

#include "chrono/physics/ChGlobal.h"
#include "chrono/physics/ChProbe.h"

namespace chrono {

ChProbe::ChProbe() {
    // mark with unique ID
    SetIdentifier(GetUniqueIntID());
}

ChProbe::ChProbe(const ChProbe& other) : ChObj(other) {}

void ChProbe::Copy(ChProbe* source) {
    // first copy the parent class data...
    ChObj::Copy(source);

    // copy other data..
}

}  // end namespace chrono
