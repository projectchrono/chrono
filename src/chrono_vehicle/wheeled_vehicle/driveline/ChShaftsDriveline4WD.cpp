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
//
// 4WD driveline model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"

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
ChShaftsDriveline4WD::ChShaftsDriveline4WD(const std::string& name)
    : ChDrivelineWV(name), m_dir_motor_block(ChVector<>(1, 0, 0)), m_dir_axle(ChVector<>(0, 1, 0)) {}

ChShaftsDriveline4WD::~ChShaftsDriveline4WD() {
    auto sys = m_central_differential->GetSystem();
    if (sys) {
        sys->Remove(m_driveshaft);
        sys->Remove(m_central_differential);
        sys->Remove(m_central_clutch);
        sys->Remove(m_front_shaft);
        sys->Remove(m_rear_shaft);
        sys->Remove(m_rear_conicalgear);
        sys->Remove(m_rear_differential);
        sys->Remove(m_rear_differentialbox);
        sys->Remove(m_rear_clutch);
        sys->Remove(m_front_conicalgear);
        sys->Remove(m_front_differential);
        sys->Remove(m_front_differentialbox);
        sys->Remove(m_front_clutch);
    }
}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void ChShaftsDriveline4WD::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChAxleList& axles,
                                      const std::vector<int>& driven_axles) {
    ChDriveline::Initialize(chassis);

    assert(axles.size() >= 2);
    assert(driven_axles.size() == 2);

    m_driven_axles = driven_axles;

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Create the driveshaft for the connection of the driveline to the transmission box.
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    sys->AddShaft(m_driveshaft);

    // Create the shaft connecting the central differential to the front differential.
    m_front_shaft = chrono_types::make_shared<ChShaft>();
    m_front_shaft->SetInertia(GetToFrontDiffShaftInertia());
    sys->AddShaft(m_front_shaft);

    // Create the shaft that connecting the central differential to the rear differential.
    m_rear_shaft = chrono_types::make_shared<ChShaft>();
    m_rear_shaft->SetInertia(GetToRearDiffShaftInertia());
    sys->AddShaft(m_rear_shaft);

    // Create the central differential, i.e. an epicycloidal mechanism that connects three rotating members.
    // A proper 'ordinary' transmission ratio t0 must be set according to Willis formula. For a differential, t0=-1.
    m_central_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_central_differential->Initialize(m_driveshaft, m_rear_shaft, m_front_shaft);
    m_central_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_central_differential);

    // Create the clutch for central differential locking. By default, unlocked.
    m_central_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_central_clutch->Initialize(m_rear_shaft, m_front_shaft);
    m_central_clutch->SetTorqueLimit(GetCentralDifferentialLockingLimit());
    m_central_clutch->SetModulation(0);
    sys->Add(m_central_clutch);

    // ---Rear differential and axles:

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_rear_differentialbox = chrono_types::make_shared<ChShaft>();
    m_rear_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    sys->AddShaft(m_rear_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_rear_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_rear_conicalgear->Initialize(m_rear_shaft, m_rear_differentialbox, chassisBody, m_dir_motor_block, m_dir_axle);
    m_rear_conicalgear->SetTransmissionRatio(-GetRearConicalGearRatio());
    sys->Add(m_rear_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_rear_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_rear_differential->Initialize(m_rear_differentialbox, axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT),
                                    axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT));
    m_rear_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_rear_differential);

    // Create the clutch for rear differential locking. By default, unlocked.
    m_rear_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_rear_clutch->Initialize(axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT),
                              axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT));
    m_rear_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_rear_clutch->SetModulation(0);
    sys->Add(m_rear_clutch);

    // ---Front differential and axles:

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_front_differentialbox = chrono_types::make_shared<ChShaft>();
    m_front_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    sys->AddShaft(m_front_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_front_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_front_conicalgear->Initialize(m_front_shaft, m_front_differentialbox, chassisBody, m_dir_motor_block, m_dir_axle);
    m_front_conicalgear->SetTransmissionRatio(-GetFrontConicalGearRatio());
    sys->Add(m_front_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_front_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_front_differential->Initialize(m_front_differentialbox, axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT),
                                     axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT));
    m_front_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_front_differential);

    // Create the clutch for front differential locking. By default, unlocked.
    m_front_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_front_clutch->Initialize(axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT),
                               axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT));
    m_front_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_front_clutch->SetModulation(0);
    sys->Add(m_front_clutch);

    // ---Initialize shaft angular velocities based on the initial wheel angular velocities.

    double omega_axle_FL = axles[m_driven_axles[0]]->m_suspension->GetAxleSpeed(LEFT);
    double omega_axle_FR = axles[m_driven_axles[0]]->m_suspension->GetAxleSpeed(RIGHT);
    double omega_axle_RL = axles[m_driven_axles[1]]->m_suspension->GetAxleSpeed(LEFT);
    double omega_axle_RR = axles[m_driven_axles[1]]->m_suspension->GetAxleSpeed(RIGHT);

    // Front differential 
    double omega_front_differentialbox = 0.5 * (omega_axle_FL + omega_axle_FR);
    m_front_differentialbox->SetPos_dt(omega_front_differentialbox);

    // Rear differential
    double omega_rear_differentialbox = 0.5 * (omega_axle_RL + omega_axle_RR);
    m_rear_differentialbox->SetPos_dt(omega_rear_differentialbox);

    // Front conical gear
    double omega_front_shaft = omega_front_differentialbox / GetFrontConicalGearRatio();
    m_front_shaft->SetPos_dt(omega_front_shaft);

    // Rear conical gear
    double omega_rear_shaft = omega_rear_differentialbox / GetRearConicalGearRatio();
    m_rear_shaft->SetPos_dt(omega_rear_shaft);

    // Central differential
    double omega_driveshaft = 0.5 * (omega_front_shaft + omega_rear_shaft);
    m_driveshaft->SetPos_dt(omega_driveshaft);
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline4WD::Synchronize(double time, const DriverInputs& driver_inputs, double driveshaft_torque) {
    m_driveshaft->SetAppliedTorque(driveshaft_torque);
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline4WD::LockAxleDifferential(int axle, bool lock) {
    if (axle == m_driven_axles[0]) {
        m_front_clutch->SetModulation(lock ? 1 : 0);
        return;
    } else if (axle == m_driven_axles[1]) {
        m_rear_clutch->SetModulation(lock ? 1 : 0);
        return;
    } else if (axle == -1) {
        m_front_clutch->SetModulation(lock ? 1 : 0);
        m_rear_clutch->SetModulation(lock ? 1 : 0);
        return;
    }

    GetLog() << "WARNING: Incorrect axle specification in ChShaftsDriveline4WD::LockAxleDifferential.\n";
    GetLog() << "         Driven axles are: " << m_driven_axles[0] << " and " << m_driven_axles[1] << "\n";
}

void ChShaftsDriveline4WD::LockCentralDifferential(int which, bool lock) {
    m_central_clutch->SetModulation(lock ? 1 : 0);
}

// -----------------------------------------------------------------------------
double ChShaftsDriveline4WD::GetSpindleTorque(int axle, VehicleSide side) const {
    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_front_differential->GetTorqueReactionOn2() - m_front_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_front_differential->GetTorqueReactionOn3() - m_front_clutch->GetTorqueReactionOn2();
        }
    }

    if (axle == m_driven_axles[1]) {
        switch (side) {
            case LEFT:
                return -m_rear_differential->GetTorqueReactionOn2() - m_rear_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_rear_differential->GetTorqueReactionOn3() - m_rear_clutch->GetTorqueReactionOn2();
        }
    }

    return 0;
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline4WD::Disconnect() {
    m_front_differential->SetDisabled(true);
    m_rear_differential->SetDisabled(true);
    m_front_clutch->SetDisabled(true);
    m_rear_clutch->SetDisabled(true);
}

}  // end namespace vehicle
}  // end namespace chrono
