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
    : ChDriveline(name), m_dir_motor_block(ChVector<>(1, 0, 0)), m_dir_axle(ChVector<>(0, 1, 0)) {}

// -----------------------------------------------------------------------------
// Initialize the driveline subsystem.
// This function connects this driveline subsystem to the axles of the specified
// suspension subsystems.
// -----------------------------------------------------------------------------
void ChShaftsDriveline4WD::Initialize(std::shared_ptr<ChBody> chassis,
                                      const ChSuspensionList& suspensions,
                                      const std::vector<int>& driven_axles) {
    assert(suspensions.size() >= 2);
    assert(driven_axles.size() == 2);

    m_driven_axles = driven_axles;

    ChSystem* my_system = chassis->GetSystem();

    // Create the driveshaft, a 1 d.o.f. object with rotational inertia which
    // represents the connection of the driveline to the transmission box.
    m_driveshaft = std::make_shared<ChShaft>();
    m_driveshaft->SetInertia(GetDriveshaftInertia());
    my_system->Add(m_driveshaft);

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to front
    // differential.
    m_front_shaft = std::make_shared<ChShaft>();
    m_front_shaft->SetInertia(GetToFrontDiffShaftInertia());
    my_system->Add(m_front_shaft);

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the shaft that connecting central differential to rear
    // differential.
    m_rear_shaft = std::make_shared<ChShaft>();
    m_rear_shaft->SetInertia(GetToRearDiffShaftInertia());
    my_system->Add(m_rear_shaft);

    // Create the central differential, i.e. an epicycloidal mechanism that
    // connects three rotating members. This class of mechanisms can be simulated
    // using ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. The case of the differential is
    // simple: t0=-1.
    m_central_differential = std::make_shared<ChShaftsPlanetary>();
    m_central_differential->Initialize(m_driveshaft,  // the carrier
                                       m_rear_shaft, m_front_shaft);
    m_central_differential->SetTransmissionRatioOrdinary(GetCentralDifferentialRatio());
    my_system->Add(m_central_differential);

    // ---Rear differential and axles:

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_rear_differentialbox = std::make_shared<ChShaft>();
    m_rear_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    my_system->Add(m_rear_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_rear_conicalgear = std::make_shared<ChShaftsGearboxAngled>();
    m_rear_conicalgear->Initialize(m_rear_shaft, m_rear_differentialbox, chassis, m_dir_motor_block, m_dir_axle);
    m_rear_conicalgear->SetTransmissionRatio(GetRearConicalGearRatio());
    my_system->Add(m_rear_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. The case of the differential is
    // simple: t0=-1.
    m_rear_differential = std::make_shared<ChShaftsPlanetary>();
    m_rear_differential->Initialize(m_rear_differentialbox, suspensions[m_driven_axles[1]]->GetAxle(LEFT),
                                    suspensions[m_driven_axles[1]]->GetAxle(RIGHT));
    m_rear_differential->SetTransmissionRatioOrdinary(GetRearDifferentialRatio());
    my_system->Add(m_rear_differential);

    // ---Front differential and axles:

    // Create a 1 d.o.f. object: a 'shaft' with rotational inertia.
    // This represents the inertia of the rotating box of the differential.
    m_front_differentialbox = std::make_shared<ChShaft>();
    m_front_differentialbox->SetInertia(GetRearDifferentialBoxInertia());
    my_system->Add(m_front_differentialbox);

    // Create an angled gearbox, i.e a transmission ratio constraint between two
    // non parallel shafts. This is the case of the 90° bevel gears in the
    // differential. Note that, differently from the basic ChShaftsGear, this also
    // provides the possibility of transmitting a reaction torque to the box
    // (the truss).
    m_front_conicalgear = std::make_shared<ChShaftsGearboxAngled>();
    m_front_conicalgear->Initialize(m_front_shaft, m_front_differentialbox, chassis, m_dir_motor_block, m_dir_axle);
    m_front_conicalgear->SetTransmissionRatio(GetFrontConicalGearRatio());
    my_system->Add(m_front_conicalgear);

    // Create a differential, i.e. an epicycloidal mechanism that connects three
    // rotating members. This class of mechanisms can be simulated using
    // ChShaftsPlanetary; a proper 'ordinary' transmission ratio t0 must be
    // assigned according to Willis formula. The case of the differential is
    // simple: t0=-1.
    m_front_differential = std::make_shared<ChShaftsPlanetary>();
    m_front_differential->Initialize(m_front_differentialbox, suspensions[m_driven_axles[0]]->GetAxle(LEFT),
                                     suspensions[m_driven_axles[0]]->GetAxle(RIGHT));
    m_front_differential->SetTransmissionRatioOrdinary(GetFrontDifferentialRatio());
    my_system->Add(m_front_differential);

    // ---Initialize shaft angular velocities based on the initial wheel angular velocities.

    double omega_axle_FL = suspensions[m_driven_axles[0]]->GetAxleSpeed(LEFT);
    double omega_axle_FR = suspensions[m_driven_axles[0]]->GetAxleSpeed(RIGHT);
    double omega_axle_RL = suspensions[m_driven_axles[1]]->GetAxleSpeed(LEFT);
    double omega_axle_RR = suspensions[m_driven_axles[1]]->GetAxleSpeed(RIGHT);

    // Front differential 
    //// TODO : Note that we assume here that the front diff ratio = -1.
    ////        This is how it should always be anyway ->  MUST MODIFY TEMPLATE
    ////        REMOVE GetFrontDifferentialRatio() and GetRearDifferentialRatio()
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
// -----------------------------------------------------------------------------
double ChShaftsDriveline4WD::GetWheelTorque(const WheelID& wheel_id) const {
    if (wheel_id.axle() == m_driven_axles[0]) {
        switch (wheel_id.side()) {
            case LEFT:
                return -m_front_differential->GetTorqueReactionOn2();
            case RIGHT:
                return -m_front_differential->GetTorqueReactionOn3();
        }
    }

    if (wheel_id.axle() == m_driven_axles[1]) {
        switch (wheel_id.side()) {
            case LEFT:
                return -m_rear_differential->GetTorqueReactionOn2();
            case RIGHT:
                return -m_rear_differential->GetTorqueReactionOn3();
        }
    }

    return 0;
}

}  // end namespace vehicle
}  // end namespace chrono
