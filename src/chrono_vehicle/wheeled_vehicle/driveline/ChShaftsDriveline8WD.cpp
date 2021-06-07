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
// 8WD driveline model template based on ChShaft objects.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"

#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline8WD.h"

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
ChShaftsDriveline8WD::ChShaftsDriveline8WD(const std::string& name)
    : ChDrivelineWV(name), m_dir_motor_block(ChVector<>(1, 0, 0)), m_dir_axle(ChVector<>(0, 1, 0)) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline to the specified axles.
// -----------------------------------------------------------------------------
void ChShaftsDriveline8WD::Initialize(std::shared_ptr<ChChassis> chassis,
                                      const ChAxleList& axles,
                                      const std::vector<int>& driven_axles) {
    assert(axles.size() >= 4);
    assert(driven_axles.size() == 4);

    m_driven_axles = driven_axles;

    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    CreateShafts(chassis);

    CreateDifferentials(chassis, axles);

    CreateClutches(chassis, axles);

    // ---Initialize shaft angular velocities based on the initial wheel angular velocities.

    double omega_axle_FL1 = axles[m_driven_axles[0]]->m_suspension->GetAxleSpeed(LEFT);
    double omega_axle_FR1 = axles[m_driven_axles[0]]->m_suspension->GetAxleSpeed(RIGHT);

    double omega_axle_FL2 = axles[m_driven_axles[1]]->m_suspension->GetAxleSpeed(LEFT);
    double omega_axle_FR2 = axles[m_driven_axles[1]]->m_suspension->GetAxleSpeed(RIGHT);

    double omega_axle_RL1 = axles[m_driven_axles[2]]->m_suspension->GetAxleSpeed(LEFT);
    double omega_axle_RR1 = axles[m_driven_axles[2]]->m_suspension->GetAxleSpeed(RIGHT);

    double omega_axle_RL2 = axles[m_driven_axles[3]]->m_suspension->GetAxleSpeed(LEFT);
    double omega_axle_RR2 = axles[m_driven_axles[3]]->m_suspension->GetAxleSpeed(RIGHT);

    // Front1 differential
    double omega_front1_differentialbox = 0.5 * (omega_axle_FL1 + omega_axle_FR1);
    m_front1_differentialbox->SetPos_dt(omega_front1_differentialbox);
    // Front1 conical gear
    double omega_front1_shaft = omega_front1_differentialbox / GetFrontConicalGearRatio();
    m_front1_shaft->SetPos_dt(omega_front1_shaft);

    // Front 2 differential
    double omega_front2_differentialbox = 0.5 * (omega_axle_FL2 + omega_axle_FR2);
    m_front2_differentialbox->SetPos_dt(omega_front2_differentialbox);
    // Front2 conical gear
    double omega_front2_shaft = omega_front2_differentialbox / GetFrontConicalGearRatio();
    m_front2_shaft->SetPos_dt(omega_front2_shaft);

    // Rear 1 differential
    double omega_rear1_differentialbox = 0.5 * (omega_axle_RL1 + omega_axle_RR1);
    m_rear1_differentialbox->SetPos_dt(omega_rear1_differentialbox);
    // Rear 1 conical gear
    double omega_rear1_shaft = omega_rear1_differentialbox / GetRearConicalGearRatio();
    m_rear1_shaft->SetPos_dt(omega_rear1_shaft);

    // Rear 2 differential
    double omega_rear2_differentialbox = 0.5 * (omega_axle_RL2 + omega_axle_RR2);
    m_rear2_differentialbox->SetPos_dt(omega_rear2_differentialbox);
    // Rear 1 conical gear
    double omega_rear2_shaft = omega_rear2_differentialbox / GetRearConicalGearRatio();
    m_rear2_shaft->SetPos_dt(omega_rear2_shaft);

    // Frontgroup differential
    double omega_frontgroup_shaft = 0.5 * (omega_front1_shaft + omega_front2_shaft);
    m_frontgroup_shaft->SetPos_dt(omega_frontgroup_shaft);

    // Reargroup differential
    double omega_reargroup_shaft = 0.5 * (omega_rear1_shaft + omega_rear2_shaft);
    m_reargroup_shaft->SetPos_dt(omega_reargroup_shaft);

    // Central differential
    double omega_driveshaft = 0.5 * (omega_frontgroup_shaft + omega_reargroup_shaft);
    m_driveshaft->SetPos_dt(omega_driveshaft);
}

//------------------------------------------------------------------------------
void ChShaftsDriveline8WD::CreateShafts(std::shared_ptr<ChChassis> chassis) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Input Shaft
    // Create the driveshaft, a 1 d.o.f. object with rotational inertia which
    // represents the connection of the driveline to the transmission box.
    m_driveshaft = chrono_types::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    sys->Add(m_driveshaft);

    // shaft connecting frontgroup differential and central differential
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to front
    // differential.
    m_frontgroup_shaft = chrono_types::make_shared<ChShaft>();
    m_frontgroup_shaft->SetInertia(GetToFrontDiffShaftInertia());
    sys->Add(m_frontgroup_shaft);

    // shaft connecting frontgroup differential and front1 axle diff
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to front
    // differential.
    m_front1_shaft = chrono_types::make_shared<ChShaft>();
    m_front1_shaft->SetInertia(GetToFrontDiffShaftInertia());
    sys->Add(m_front1_shaft);

    // shaft connecting frontgroup differential and front2 axle diff
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to front
    // differential.
    m_front2_shaft = chrono_types::make_shared<ChShaft>();
    m_front2_shaft->SetInertia(GetToFrontDiffShaftInertia());
    sys->Add(m_front2_shaft);

    // shaft connecting reargroup differential and central differential
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to rear
    // differential.
    m_reargroup_shaft = chrono_types::make_shared<ChShaft>();
    m_reargroup_shaft->SetInertia(GetToRearDiffShaftInertia());
    sys->Add(m_reargroup_shaft);

    // shaft connecting reargroup differential and rear2 axle differential
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to rear
    // differential.
    m_rear1_shaft = chrono_types::make_shared<ChShaft>();
    m_rear1_shaft->SetInertia(GetToRearDiffShaftInertia());
    sys->Add(m_rear1_shaft);

    // shaft connecting reargroup differential and rear2 axle differential
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to rear
    // differential.
    m_rear2_shaft = chrono_types::make_shared<ChShaft>();
    m_rear2_shaft->SetInertia(GetToRearDiffShaftInertia());
    sys->Add(m_rear2_shaft);

    GetLog() << "Shafts created\n";
}

void ChShaftsDriveline8WD::CreateDifferentials(std::shared_ptr<ChChassis> chassis, const ChAxleList& axles) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Create the central differential, i.e. an epicycloidal mechanism that
    // connects three rotating members. This class of mechanisms can be simulated
    // using ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_central_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_central_differential->Initialize(m_driveshaft,  // the carrier
                                       m_reargroup_shaft, m_frontgroup_shaft);
    m_central_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_central_differential);

    // Create the frontgroup differential, i.e. an epicycloidal mechanism that
    // connects three rotating members. This class of mechanisms can be simulated
    // using ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_frontgroup_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_frontgroup_differential->Initialize(m_frontgroup_shaft,  // the carrier
                                          m_front2_shaft, m_front1_shaft);
    m_frontgroup_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_frontgroup_differential);

    // Create the reargroup differential, i.e. an epicycloidal mechanism that
    // connects three rotating members. This class of mechanisms can be simulated
    // using ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_reargroup_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_reargroup_differential->Initialize(m_reargroup_shaft,  // the carrier
                                         m_rear2_shaft, m_rear1_shaft);
    m_reargroup_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_reargroup_differential);

    GetLog() << "Distributing Differentials created\n";

    // Front 1 Axle differential ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_front1_differentialbox = chrono_types::make_shared<ChShaft>();
    m_front1_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    sys->Add(m_front1_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_front1_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_front1_conicalgear->Initialize(m_front1_shaft, m_front1_differentialbox, chassisBody, m_dir_motor_block,
                                     m_dir_axle);
    m_front1_conicalgear->SetTransmissionRatio(-GetRearConicalGearRatio());
    sys->Add(m_front1_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_front1_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_front1_differential->Initialize(m_front1_differentialbox, axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT),
                                      axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT));
    m_front1_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_front1_differential);

    // Front 2 Axle differential ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_front2_differentialbox = chrono_types::make_shared<ChShaft>();
    m_front2_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    sys->Add(m_front2_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_front2_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_front2_conicalgear->Initialize(m_front2_shaft, m_front2_differentialbox, chassisBody, m_dir_motor_block,
                                     m_dir_axle);
    m_front2_conicalgear->SetTransmissionRatio(-GetRearConicalGearRatio());
    sys->Add(m_front2_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_front2_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_front2_differential->Initialize(m_front1_differentialbox, axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT),
                                      axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT));
    m_front2_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_front2_differential);

    // Rear 1 Axle differential ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_rear1_differentialbox = chrono_types::make_shared<ChShaft>();
    m_rear1_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    sys->Add(m_rear1_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_rear1_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_rear1_conicalgear->Initialize(m_rear1_shaft, m_rear1_differentialbox, chassisBody, m_dir_motor_block, m_dir_axle);
    m_rear1_conicalgear->SetTransmissionRatio(-GetRearConicalGearRatio());
    sys->Add(m_rear1_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_rear1_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_rear1_differential->Initialize(m_rear1_differentialbox, axles[m_driven_axles[2]]->m_suspension->GetAxle(LEFT),
                                     axles[m_driven_axles[2]]->m_suspension->GetAxle(RIGHT));
    m_rear1_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_rear1_differential);

    // Rear 2 Axle differential ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_rear2_differentialbox = chrono_types::make_shared<ChShaft>();
    m_rear2_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    sys->Add(m_rear2_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_rear2_conicalgear = chrono_types::make_shared<ChShaftsGearboxAngled>();
    m_rear2_conicalgear->Initialize(m_rear2_shaft, m_rear2_differentialbox, chassisBody, m_dir_motor_block, m_dir_axle);
    m_rear2_conicalgear->SetTransmissionRatio(-GetRearConicalGearRatio());
    sys->Add(m_rear2_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. For a differential, t0=-1.
    m_rear2_differential = chrono_types::make_shared<ChShaftsPlanetary>();
    m_rear2_differential->Initialize(m_rear2_differentialbox, axles[m_driven_axles[3]]->m_suspension->GetAxle(LEFT),
                                     axles[m_driven_axles[3]]->m_suspension->GetAxle(RIGHT));
    m_rear2_differential->SetTransmissionRatioOrdinary(-1.0);
    sys->Add(m_rear2_differential);

    GetLog() << "Axle Differentials created\n";
}

//-----------------------------------------------------------------------------
void ChShaftsDriveline8WD::CreateClutches(std::shared_ptr<ChChassis> chassis, const ChAxleList& axles) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Create the clutch for central differential locking. By default, unlocked.
    m_central_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_central_clutch->Initialize(m_reargroup_shaft, m_frontgroup_shaft);
    m_central_clutch->SetTorqueLimit(GetCentralDifferentialLockingLimit());
    m_central_clutch->SetModulation(0);
    sys->Add(m_central_clutch);

    // Create the clutch for frontgroup differential locking. By default, unlocked.
    m_frontgroup_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_frontgroup_clutch->Initialize(m_front2_shaft, m_front1_shaft);
    m_frontgroup_clutch->SetTorqueLimit(GetCentralDifferentialLockingLimit());
    m_frontgroup_clutch->SetModulation(0);
    sys->Add(m_frontgroup_clutch);

    // Create the clutch for reargroup differential locking. By default, unlocked.
    m_reargroup_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_reargroup_clutch->Initialize(m_rear2_shaft, m_rear1_shaft);
    m_reargroup_clutch->SetTorqueLimit(GetCentralDifferentialLockingLimit());
    m_reargroup_clutch->SetModulation(0);
    sys->Add(m_reargroup_clutch);

    // Create the clutch for front1 differential locking. By default, unlocked.
    m_front1_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_front1_clutch->Initialize(axles[m_driven_axles[0]]->m_suspension->GetAxle(LEFT),
                                axles[m_driven_axles[0]]->m_suspension->GetAxle(RIGHT));
    m_front1_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_front1_clutch->SetModulation(0);
    sys->Add(m_front1_clutch);

    // Create the clutch for front2 differential locking. By default, unlocked.
    m_front2_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_front2_clutch->Initialize(axles[m_driven_axles[1]]->m_suspension->GetAxle(LEFT),
                                axles[m_driven_axles[1]]->m_suspension->GetAxle(RIGHT));
    m_front2_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_front2_clutch->SetModulation(0);
    sys->Add(m_front2_clutch);

    // Create the clutch for rear1 differential locking. By default, unlocked.
    m_rear1_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_rear1_clutch->Initialize(axles[m_driven_axles[2]]->m_suspension->GetAxle(LEFT),
                               axles[m_driven_axles[2]]->m_suspension->GetAxle(RIGHT));
    m_rear1_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_rear1_clutch->SetModulation(0);
    sys->Add(m_rear1_clutch);

    // Create the clutch for rear2 differential locking. By default, unlocked.
    m_rear2_clutch = chrono_types::make_shared<ChShaftsClutch>();
    m_rear2_clutch->Initialize(axles[m_driven_axles[3]]->m_suspension->GetAxle(LEFT),
                               axles[m_driven_axles[3]]->m_suspension->GetAxle(RIGHT));
    m_rear2_clutch->SetTorqueLimit(GetAxleDifferentialLockingLimit());
    m_rear2_clutch->SetModulation(0);
    sys->Add(m_rear2_clutch);

    GetLog() << "Locking Clutches created\n";
}

// -----------------------------------------------------------------------------
void ChShaftsDriveline8WD::LockAxleDifferential(int axle, bool lock) {
    if (axle == m_driven_axles[0]) {
        m_front1_clutch->SetModulation(lock ? 1 : 0);
        return;
    } else if (axle == m_driven_axles[1]) {
        m_front1_clutch->SetModulation(lock ? 1 : 0);
        return;
    } else if (axle == m_driven_axles[2]) {
        m_rear1_clutch->SetModulation(lock ? 1 : 0);
        return;
    } else if (axle == m_driven_axles[3]) {
        m_rear2_clutch->SetModulation(lock ? 1 : 0);
        return;
    } else if (axle == -1) {
        m_front1_clutch->SetModulation(lock ? 1 : 0);
        m_front2_clutch->SetModulation(lock ? 1 : 0);
        m_rear1_clutch->SetModulation(lock ? 1 : 0);
        m_rear2_clutch->SetModulation(lock ? 1 : 0);
        return;
    }

    // GetLog() << "WARNING: Incorrect axle specification in ChShaftsDriveline8WD::LockAxleDifferential.\n";
    // GetLog() << "         Driven axles are: " << m_driven_axles[0] << " and " << m_driven_axles[1] << "\n";
}

void ChShaftsDriveline8WD::LockCentralDifferential(int which, bool lock) {
    switch (which) {
        case 0:
            m_frontgroup_clutch->SetModulation(lock ? 1 : 0);
            return;
        case 1:
            m_reargroup_clutch->SetModulation(lock ? 1 : 0);
            return;
        case 2:
            m_central_clutch->SetModulation(lock ? 1 : 0);
            return;
        case -1:
            m_frontgroup_clutch->SetModulation(lock ? 1 : 0);
            m_reargroup_clutch->SetModulation(lock ? 1 : 0);
            m_central_clutch->SetModulation(lock ? 1 : 0);
            return;
    }
}

// -----------------------------------------------------------------------------
double ChShaftsDriveline8WD::GetSpindleTorque(int axle, VehicleSide side) const {
    if (axle == m_driven_axles[0]) {
        switch (side) {
            case LEFT:
                return -m_front1_differential->GetTorqueReactionOn2() - m_front1_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_front1_differential->GetTorqueReactionOn3() - m_front1_clutch->GetTorqueReactionOn2();
        }
    }

    if (axle == m_driven_axles[1]) {
        switch (side) {
            case LEFT:
                return -m_front2_differential->GetTorqueReactionOn2() - m_front2_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_front2_differential->GetTorqueReactionOn3() - m_front2_clutch->GetTorqueReactionOn2();
        }
    }

    if (axle == m_driven_axles[2]) {
        switch (side) {
            case LEFT:
                return -m_rear1_differential->GetTorqueReactionOn2() - m_rear1_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_rear1_differential->GetTorqueReactionOn3() - m_rear1_clutch->GetTorqueReactionOn2();
        }
    }

    if (axle == m_driven_axles[3]) {
        switch (side) {
            case LEFT:
                return -m_rear2_differential->GetTorqueReactionOn2() - m_rear2_clutch->GetTorqueReactionOn1();
            case RIGHT:
                return -m_rear2_differential->GetTorqueReactionOn3() - m_rear2_clutch->GetTorqueReactionOn2();
        }
    }

    return 0;
}

}  // end namespace vehicle
}  // end namespace chrono
