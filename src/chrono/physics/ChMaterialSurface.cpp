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

#include "chrono/core/ChClassFactory.h"
#include "chrono/physics/ChMaterialSurface.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChMaterialSurface)

ChMaterialSurface::ChMaterialSurface()
    : static_friction(0.6f),
      sliding_friction(0.6f),
      rolling_friction(0),
      spinning_friction(0),
      restitution(0),
      cohesion(0),
      dampingf(0),
      compliance(0),
      complianceT(0),
      complianceRoll(0),
      complianceSpin(0) {}

ChMaterialSurface::ChMaterialSurface(const ChMaterialSurface& other) {
    static_friction = other.static_friction;
    sliding_friction = other.sliding_friction;
    rolling_friction = other.rolling_friction;
    spinning_friction = other.spinning_friction;
    restitution = other.restitution;
    cohesion = other.cohesion;
    dampingf = other.dampingf;
    compliance = other.compliance;
    complianceT = other.complianceT;
    complianceRoll = other.complianceRoll;
    complianceSpin = other.complianceSpin;
}

void ChMaterialSurface::SetFriction(float mval) {
    SetSfriction(mval);
    SetKfriction(mval);
}

void ChMaterialSurface::ArchiveOUT(ChArchiveOut& marchive){
    // version number
    marchive.VersionWrite<ChMaterialSurface>();

    // serialize parent class
    ChMaterialSurfaceBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(static_friction);
    marchive << CHNVP(sliding_friction);
    marchive << CHNVP(rolling_friction);
    marchive << CHNVP(spinning_friction);
    marchive << CHNVP(restitution);
    marchive << CHNVP(cohesion);
    marchive << CHNVP(dampingf);
    marchive << CHNVP(compliance);
    marchive << CHNVP(complianceT);
    marchive << CHNVP(complianceRoll);
    marchive << CHNVP(complianceSpin);
}

void ChMaterialSurface::ArchiveIN(ChArchiveIn& marchive){
    // version number
    int version = marchive.VersionRead<ChMaterialSurface>();

    // deserialize parent class
    ChMaterialSurfaceBase::ArchiveIN(marchive);

    // stream in all member data:
    marchive >> CHNVP(static_friction);
    marchive >> CHNVP(sliding_friction);
    marchive >> CHNVP(rolling_friction);
    marchive >> CHNVP(spinning_friction);
    marchive >> CHNVP(restitution);
    marchive >> CHNVP(cohesion);
    marchive >> CHNVP(dampingf);
    marchive >> CHNVP(compliance);
    marchive >> CHNVP(complianceT);
    marchive >> CHNVP(complianceRoll);
    marchive >> CHNVP(complianceSpin);
}

}  // end namespace chrono
