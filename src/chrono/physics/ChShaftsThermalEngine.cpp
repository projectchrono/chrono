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

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsThermalEngine.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChShaftsThermalEngine)

ChShaftsThermalEngine::ChShaftsThermalEngine() : throttle(1), error_backward(false) {
    // default torque curve= constant zero. User will provide better fx.
    Tw = chrono_types::make_shared<ChFunctionConst>(0);
}

ChShaftsThermalEngine::ChShaftsThermalEngine(const ChShaftsThermalEngine& other) : ChShaftsTorque(other) {
    throttle = other.throttle;
    error_backward = other.error_backward;
    Tw = std::shared_ptr<ChFunction>(other.Tw->Clone());  // deep copy
}

double ChShaftsThermalEngine::ComputeTorque() {
    // COMPUTE THE TORQUE HERE!
    double mw = GetRelativePosDt();

    if (mw < 0)
        error_backward = true;
    else
        error_backward = false;

    // get the actual torque from torque curve
    double mT = Tw->GetVal(mw);

    // modulate it with throttle
    double modulated_T = mT * throttle;

    return modulated_T;
}

void ChShaftsThermalEngine::ArchiveOut(ChArchiveOut& archive_out) {
    // version number
    archive_out.VersionWrite<ChShaftsThermalEngine>();

    // serialize parent class
    ChShaftsTorque::ArchiveOut(archive_out);

    // serialize all member data:
    archive_out << CHNVP(Tw);
    archive_out << CHNVP(throttle);
}

/// Method to allow de serialization of transient data from archives.
void ChShaftsThermalEngine::ArchiveIn(ChArchiveIn& archive_in) {
    // version number
    /*int version =*/archive_in.VersionRead<ChShaftsThermalEngine>();

    // deserialize parent class:
    ChShaftsTorque::ArchiveIn(archive_in);

    // deserialize all member data:
    archive_in >> CHNVP(Tw);
    archive_in >> CHNVP(throttle);
}

}  // end namespace chrono