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

#include <algorithm>
#include <cmath>

#include "chrono/physics/ChMaterialSurface.h"
#include "chrono/physics/ChMaterialSurfaceSMC.h"
#include "chrono/physics/ChMaterialSurfaceNSC.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// -----------------------------------------------------------------------------

ChMaterialSurface::ChMaterialSurface()
    : static_friction(0.6f),
      sliding_friction(0.6f),
      rolling_friction(0),
      spinning_friction(0),
      restitution(0.4f) {}

ChMaterialSurface::ChMaterialSurface(const ChMaterialSurface& other) {
    static_friction = other.static_friction;
    sliding_friction = other.sliding_friction;
    rolling_friction = other.rolling_friction;
    spinning_friction = other.spinning_friction;
    restitution = other.restitution;
}

void ChMaterialSurface::SetFriction(float val) {
    SetSfriction(val);
    SetKfriction(val);
}

void ChMaterialSurface::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChMaterialSurface>();

    // serialize all member data:
    marchive << CHNVP(static_friction);
    marchive << CHNVP(sliding_friction);
    marchive << CHNVP(rolling_friction);
    marchive << CHNVP(spinning_friction);
    marchive << CHNVP(restitution);
}

void ChMaterialSurface::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChMaterialSurface>();

    // stream in all member data:
    marchive >> CHNVP(static_friction);
    marchive >> CHNVP(sliding_friction);
    marchive >> CHNVP(rolling_friction);
    marchive >> CHNVP(spinning_friction);
    marchive >> CHNVP(restitution);
}

std::shared_ptr<ChMaterialSurface> ChMaterialSurface::DefaultMaterial(ChContactMethod contact_method) {
    switch (contact_method) {
        case ChContactMethod::NSC:
            return chrono_types::make_shared<ChMaterialSurfaceNSC>();
        case ChContactMethod::SMC:
            return chrono_types::make_shared<ChMaterialSurfaceSMC>();
    }
    return nullptr;
}

}  // end namespace chrono
