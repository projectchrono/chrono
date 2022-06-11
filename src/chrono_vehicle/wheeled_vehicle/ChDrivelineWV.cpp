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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a wheeled vehicle driveline.
//
// =============================================================================

#include "chrono_vehicle/wheeled_vehicle/ChDrivelineWV.h"

namespace chrono {
namespace vehicle {

ChDrivelineWV::ChDrivelineWV(const std::string& name) : ChDriveline(name) {}

void ChDrivelineWV::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    m_driveshaft->SetAppliedTorque(torque);
}

void ChDrivelineWV::LockAxleDifferential(int axle, bool lock) {
    GetLog() << "WARNING: Differential locking not yet implemented for " << GetTemplateName() << "\n";
}

void ChDrivelineWV::LockCentralDifferential(int which, bool lock) {
    GetLog() << "WARNING: Differential locking not yet implemented for " << GetTemplateName() << "\n";
}

}  // end namespace vehicle
}  // end namespace chrono
