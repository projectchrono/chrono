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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsThermalEngine.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsThermalEngine)

ChShaftsThermalEngine::ChShaftsThermalEngine() : throttle(1), error_backward(false) {
    // default torque curve= constant zero. User will provide better fx.
    Tw = std::make_shared<ChFunction_Const>(0);
}

ChShaftsThermalEngine::ChShaftsThermalEngine(const ChShaftsThermalEngine& other) : ChShaftsTorqueBase(other) {
    throttle = other.throttle;
    error_backward = other.error_backward;
    Tw = std::shared_ptr<ChFunction>(other.Tw->Clone());  // deep copy
}

double ChShaftsThermalEngine::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    double mw = GetRelativeRotation_dt();

    if (mw < 0)
        error_backward = true;
    else
        error_backward = false;

    // get the actual torque from torque curve
    double mT = Tw->Get_y(mw);

    // modulate it with throttle
    double modulated_T = mT * throttle;

    return modulated_T;
}

void ChShaftsThermalEngine::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChShaftsThermalEngine>();

    // serialize parent class
    ChShaftsTorqueBase::ArchiveOUT(marchive);

    // serialize all member data:
    marchive << CHNVP(Tw);
    marchive << CHNVP(throttle);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsThermalEngine::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChShaftsThermalEngine>();

    // deserialize parent class:
    ChShaftsTorqueBase::ArchiveIN(marchive);

    // deserialize all member data:
    marchive >> CHNVP(Tw);
    marchive >> CHNVP(throttle);
}

}  // end namespace chrono