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
// Authors: Alessandro Tasora, Radu Serban, Rainer Gericke
// =============================================================================
//
// 2WD driveline model template based on ChShaft objects. This template can be
// used to model either a FWD or a RWD driveline.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline2WD.h"

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
ChShaftsDriveline2WD::ChShaftsDriveline2WD(const std::string& name)
    : ChDrivelineWV(name), m_dir_motor_block(ChVector<>(1, 0, 0)), m_dir_axle(ChVector<>(0, 1, 0)) {}

ChShaftsDriveline2WD::~ChShaftsDriveline2WD() {
    auto sys = m_differential->GetSystem();
    if (sys) {
        sys->Remove(m_driveshaft);
        sys->Remove(m_conicalgear);
        sys->Remove(m_differentialbox);
        sys->Remove(m_differential);
        sys->Remove(m_clutch);
    }
}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axle.
// -----------------------------------------------------------------------------
void ChShaftsDriveline2WD::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChAxleList& axles,
                                      const std::vector<int>& driven_axles) {
    ChDriveline::Initialize(chassis);

    assert(axles.size() >= 1);
    assert(driven_axles.size() == 1);

    m_driven_axles = driven_axles;

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Create the driveshaft for the connection of the driveline to the transmission box.
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    sys->AddShaft(m_driveshaft);

    // Create the differential box.
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
    // A proper 'ordinary' transmission ratio t0 must be set according to Willis formula. For a differential, t0=-1.
    m_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_differential->Initialize(m_differentialbox, axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT),
                               axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT));
    m_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_differential);

    // Create the clutch for differential locking. By default, unlocked.
    m_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_clutch->Initialize(axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT),
                         axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT));
    m_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_clutch->SetModulation(0);
    sys->Add(m_clutch);
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline2WD::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_torque) {
    m_driveshaft->SetAppliedTorque(driveshaft_torque);
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline2WD::LockAxleDifferential(int axle, bool lock) {
    m_clutch->SetModulation(lock ? 1 : 0);
}

void ChShaftsDriveline2WD::LockCentralDifferential(int which, bool lock) {
    GetLog() << "WARNINIG: " << GetTemplateName() << " does not contain a central differential.\n";
}

// -----------------------------------------------------------------------------
double ChShaftsDriveline2WD::GetSpindleTorque(int axle, VehicleSide side) const {
    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_differential->GetTorqueReactionOn2() - m_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_differential->GetTorqueReactionOn3() - m_clutch->GetTorqueReactionOn2();
        }
    }

    return 0;
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline2WD::Disconnect() {
    m_differential->SetDisabled(true);
    m_clutch->SetDisabled(true);
}

}  // end namespace vehicle
}  // end namespace chrono
