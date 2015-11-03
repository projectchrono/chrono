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
// Authors: Radu Serban
// =============================================================================
//
// Irrlicht-based visualization wrapper for tracked vehicles.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#include "chrono/core/ChMathematics.h"

#include "chrono_vehicle/tracked_vehicle/utils/ChTrackedVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/driveline/ChSimpleTrackDriveline.h"

using namespace irr;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTrackedVehicleIrrApp::ChTrackedVehicleIrrApp(ChVehicle* vehicle,
                                               ChPowertrain* powertrain,
                                               const wchar_t* title,
                                               irr::core::dimension2d<irr::u32> dims)
    : ChVehicleIrrApp(vehicle, powertrain, title, dims) {
    m_tvehicle = dynamic_cast<ChTrackedVehicle*>(vehicle);
    assert(m_tvehicle);
}

// -----------------------------------------------------------------------------
// Render stats for the vehicle driveline subsystem.
// -----------------------------------------------------------------------------
void ChTrackedVehicleIrrApp::renderOtherStats(int left, int top) {
    char msg[100];

    if (ChSharedPtr<ChSimpleTrackDriveline> driveline =
            m_tvehicle->GetDriveline().DynamicCastTo<ChSimpleTrackDriveline>()) {
        double toRPM = 30 / CH_C_PI;

        double shaft_speed = driveline->GetDriveshaftSpeed() * toRPM;
        sprintf(msg, "Driveshaft (RPM): %+.2f", shaft_speed);
        renderLinGauge(std::string(msg), shaft_speed / 2000, false, left, top, 120, 15);

        double speedL = driveline->GetSprocketSpeed(LEFT) * toRPM;
        sprintf(msg, "Sprocket L (RPM): %+.2f", speedL);
        renderLinGauge(std::string(msg), speedL / 2000, false, left, top + 30, 120, 15);

        double torqueL = driveline->GetSprocketTorque(LEFT);
        sprintf(msg, "Torque sprocket L: %+.2f", torqueL);
        renderLinGauge(std::string(msg), torqueL / 5000, false, left, top + 50, 120, 15);

        double speedR = driveline->GetSprocketSpeed(RIGHT) * toRPM;
        sprintf(msg, "Sprocket R (RPM): %+.2f", speedR);
        renderLinGauge(std::string(msg), speedR / 2000, false, left, top + 80, 120, 15);

        double torqueR = driveline->GetSprocketTorque(RIGHT);
        sprintf(msg, "Torque sprocket R: %+.2f", torqueR);
        renderLinGauge(std::string(msg), torqueR / 5000, false, left, top + 100, 120, 15);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
