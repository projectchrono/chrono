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
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple driveline model for a single axle open differential.
//
// =============================================================================

#include <cmath>

#include "chrono_models/vehicle/generic/Generic_SimpleDriveline.h"

namespace chrono {
namespace vehicle {
namespace generic {
    // -----------------------------------------------------------------------------
    // Static variables
    // -----------------------------------------------------------------------------
    const double Generic_SimpleDriveline::m_conicalgear_ratio = -0.2433;

    // -----------------------------------------------------------------------------
    // Construct a 2WD open differential simple driveline.
    // -----------------------------------------------------------------------------
    Generic_SimpleDriveline::Generic_SimpleDriveline(const std::string& name) : ChDriveline(name) {}

    // -----------------------------------------------------------------------------
    // Initialize the driveline subsystem.
    // This function connects this driveline subsystem to the axles of the specified
    // suspension subsystems.
    // -----------------------------------------------------------------------------
    void Generic_SimpleDriveline::Initialize(std::shared_ptr<ChBody> chassis,
        const ChSuspensionList& suspensions,
        const std::vector<int>& driven_axles) {
        assert(suspensions.size() >= 1);

        m_driven_axles = driven_axles;

        // Grab handles to the suspension wheel shafts.
        m_driven_left = suspensions[m_driven_axles[0]]->GetAxle(LEFT);
        m_driven_right = suspensions[m_driven_axles[0]]->GetAxle(RIGHT);
    }

    // -----------------------------------------------------------------------------
    // -----------------------------------------------------------------------------
    double Generic_SimpleDriveline::GetDriveshaftSpeed() const {
        double wheel_speed = 0.5 * (m_driven_left->GetPos_dt() + m_driven_right->GetPos_dt());

        return (wheel_speed / m_conicalgear_ratio);
    }

    // -----------------------------------------------------------------------------
    // -----------------------------------------------------------------------------
    void Generic_SimpleDriveline::Synchronize(double torque) {
        // Split the input torque front/back.
        double torque_drive = -torque / m_conicalgear_ratio;

        m_driven_left->SetAppliedTorque(-torque_drive / 2);
        m_driven_right->SetAppliedTorque(-torque_drive / 2);

    }

    // -----------------------------------------------------------------------------
    // -----------------------------------------------------------------------------
    double Generic_SimpleDriveline::GetWheelTorque(const WheelID& wheel_id) const {
        if (wheel_id.axle() == m_driven_axles[0]) {
            switch (wheel_id.side()) {
            case LEFT:
                return -m_driven_left->GetAppliedTorque();
            case RIGHT:
                return -m_driven_right->GetAppliedTorque();
            }
        }

        return 0;
    }

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
