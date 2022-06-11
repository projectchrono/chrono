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
// Authors:
// =============================================================================
//
// Track driveline model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/tracked_vehicle/driveline/ChTrackDrivelineBDS.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// dir_motor_block specifies the direction of the driveshaft, i.e. the input of
// the conic gear pair, in chassis local coords.
//
// dir_axle specifies the direction of the axle, i.e. the output of the conic
// conic gear pair, in chassis local coords. This is needed because ChShaftsBody
// could transfer pitch torque to the chassis.
// -----------------------------------------------------------------------------
ChTrackDrivelineBDS::ChTrackDrivelineBDS(const std::string& name)
    : ChDrivelineTV(name), m_dir_motor_block(ChVector<>(1, 0, 0)), m_dir_axle(ChVector<>(0, 1, 0)) {}

ChTrackDrivelineBDS::~ChTrackDrivelineBDS() {
    auto sys = m_differential->GetSystem();
    if (sys) {
        sys->Remove(m_differential);
        sys->Remove(m_conicalgear);
        sys->Remove(m_differentialbox);
    }
}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the axles of the specified
// suspension subsystems.
// -----------------------------------------------------------------------------
void ChTrackDrivelineBDS::Initialize(std::shared_ptr<ChChassis> chassis,
                                     std::shared_ptr<ChTrackAssembly> track_left,
                                     std::shared_ptr<ChTrackAssembly> track_right) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Create the driveshaft, a 1 d.o.f. object with rotational inertia which
    // represents the connection of the driveline to the transmission box.
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    sys->AddShaft(m_driveshaft);

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_differentialbox = chrono_types::make_shared<ChShaft>();
    m_differentialbox->SetInertia(GetDifferentialBoxInertia());
    sys->AddShaft(m_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_conicalgear->Initialize(m_driveshaft, m_differentialbox, chassisBody, m_dir_motor_block, m_dir_axle);
    m_conicalgear->SetTransmissionRatio(-GetConicalGearRatio());
    sys->Add(m_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three rotating members.
    // This class of mechanisms can be simulated using ChShaftsPlanetary; a proper 'ordinary'
    // transmission ratio t0 must be assigned according to Willis formula. For a differential, t0=-1.
    m_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_differential->Initialize(m_differentialbox, track_left->GetSprocket()->GetAxle(),
                               track_right->GetSprocket()->GetAxle());
    m_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_differential);
}

// -----------------------------------------------------------------------------
void ChTrackDrivelineBDS::CombineDriverInputs(const DriverInputs& driver_inputs,
                                              double& braking_left,
                                              double& braking_right) {
    braking_left = driver_inputs.m_braking;
    braking_right = driver_inputs.m_braking;
    if (driver_inputs.m_steering > 0) {
        braking_left += driver_inputs.m_steering;
    } else if (driver_inputs.m_steering < 0) {
        braking_right -= driver_inputs.m_steering;
    }
}

void ChTrackDrivelineBDS::Synchronize(double time, const DriverInputs& driver_inputs, double torque) {
    ChDrivelineTV::Synchronize(time, driver_inputs, torque);
}

// -----------------------------------------------------------------------------
double ChTrackDrivelineBDS::GetSprocketTorque(VehicleSide side) const {
    switch (side) {
        case LEFT:
            return -m_differential->GetTorqueReactionOn2();
        case RIGHT:
            return -m_differential->GetTorqueReactionOn3();
    }

    return 0;
}

double ChTrackDrivelineBDS::GetSprocketSpeed(VehicleSide side) const {
    switch (side) {
        case LEFT: {
            return -m_differential->GetSpeedShaft2();
        }
        case RIGHT: {
            return -m_differential->GetSpeedShaft3();
        }
    }

    return 0;
}

}  // end namespace vehicle
}  // end namespace chrono
