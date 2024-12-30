// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke
// =============================================================================
//
// Duro vehicle model.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_models/vehicle/duro/Duro_Vehicle.h"
#include "chrono_models/vehicle/duro/Duro_BrakeShafts.h"
#include "chrono_models/vehicle/duro/Duro_Chassis.h"
#include "chrono_models/vehicle/duro/Duro_Driveline4WD.h"
#include "chrono_models/vehicle/duro/Duro_DeDionAxle.h"
#include "chrono_models/vehicle/duro/Duro_RotaryArm.h"
#include "chrono_models/vehicle/duro/Duro_ToeBarDeDionAxle.h"
#include "chrono_models/vehicle/duro/Duro_Wheel.h"

namespace chrono {
namespace vehicle {
namespace duro {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Duro_Vehicle::Duro_Vehicle(const bool fixed,
                           BrakeType brake_type,
                           SteeringTypeWV steering_model,
                           ChContactMethod contact_method,
                           CollisionType chassis_collision_type)
    : ChWheeledVehicle("Duro", contact_method), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, steering_model, chassis_collision_type);
}

Duro_Vehicle::Duro_Vehicle(ChSystem* system,
                           const bool fixed,
                           BrakeType brake_type,
                           SteeringTypeWV steering_model,
                           CollisionType chassis_collision_type)
    : ChWheeledVehicle("Duro", system), m_omega({0, 0, 0, 0}) {
    Create(fixed, brake_type, steering_model, chassis_collision_type);
}

void Duro_Vehicle::Create(bool fixed,
                          BrakeType brake_type,
                          SteeringTypeWV steering_model,
                          CollisionType chassis_collision_type) {
    // Create the chassis subsystem
    m_chassis = chrono_types::make_shared<Duro_Chassis>("Chassis", fixed, chassis_collision_type);

    // Create the steering subsystem
    m_steerings.resize(1);
    m_steerings[0] = chrono_types::make_shared<Duro_RotaryArm>("Steering");

    // Create the axle subsystems
    m_axles.resize(2);
    m_axles[0] = chrono_types::make_shared<ChAxle>();
    m_axles[1] = chrono_types::make_shared<ChAxle>();

    m_axles[0]->m_suspension = chrono_types::make_shared<Duro_ToeBarDeDionAxle>("FrontSusp");
    m_axles[1]->m_suspension = chrono_types::make_shared<Duro_DeDionAxle>("RearSusp");

    m_axles[0]->m_wheels.resize(2);
    m_axles[0]->m_wheels[0] = chrono_types::make_shared<Duro_Wheel>("Wheel_FL");
    m_axles[0]->m_wheels[1] = chrono_types::make_shared<Duro_Wheel>("Wheel_FR");
    m_axles[1]->m_wheels.resize(2);
    m_axles[1]->m_wheels[0] = chrono_types::make_shared<Duro_Wheel>("Wheel_RL");
    m_axles[1]->m_wheels[1] = chrono_types::make_shared<Duro_Wheel>("Wheel_RR");

    switch (brake_type) {
        case BrakeType::SIMPLE:
            std::cout << "Buggy simple brake changed to shafts brake!\n";
        case BrakeType::SHAFTS:
            m_axles[0]->m_brake_left = chrono_types::make_shared<Duro_BrakeShafts>("Brake_FL");
            m_axles[0]->m_brake_right = chrono_types::make_shared<Duro_BrakeShafts>("Brake_FR");
            m_axles[1]->m_brake_left = chrono_types::make_shared<Duro_BrakeShafts>("Brake_RL");
            m_axles[1]->m_brake_right = chrono_types::make_shared<Duro_BrakeShafts>("Brake_RR");
            break;
    }

    // Create the driveline
    m_driveline = chrono_types::make_shared<Duro_Driveline4WD>("Driveline");
}

Duro_Vehicle::~Duro_Vehicle() {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Duro_Vehicle::Initialize(const ChCoordsys<>& chassisPos, double chassisFwdVel) {
    // Initialize the chassis subsystem.
    m_chassis->Initialize(m_system, chassisPos, chassisFwdVel, WheeledCollisionFamily::CHASSIS);

    // Initialize the steering subsystem (specify the steering subsystem's frame relative to the chassis reference
    // frame).
    ChVector3d offset = ChVector3d(0, 0, 0);
    ChQuaternion<> rotation = QuatFromAngleY(0);
    m_steerings[0]->Initialize(m_chassis, offset, rotation);

    // Initialize the axle subsystems.
    m_axles[0]->Initialize(m_chassis, nullptr, m_steerings[0], ChVector3d(0, 0, 0), ChVector3d(0), 0.0, m_omega[0],
                           m_omega[1]);
    m_axles[1]->Initialize(m_chassis, nullptr, nullptr, ChVector3d(-3.88, 0, 0), ChVector3d(0), 0.0, m_omega[2],
                           m_omega[3]);

    // special roll stabilizer mechanism *********************************************************
    auto frontsusp = std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[0]->m_suspension);
    auto rearsusp = std::static_pointer_cast<ChDeDionAxle>(m_axles[1]->m_suspension);

    ChVector3d p1L = frontsusp->GetConnectorLocation(LEFT);  // lower front rod
    ChVector3d p2L = p1L + ChVector3d(0, 0, 0.2);            // upper front rod
    ChVector3d p5L = rearsusp->GetConnectorLocation(LEFT);   // lower rear rod
    ChVector3d p4L = p5L + ChVector3d(0, 0, 0.2);            // upper rear rod
    ChVector3d p3L = (p2L + p4L) / 2;                        // beam pivot point

    ChVector3d p1R = frontsusp->GetConnectorLocation(RIGHT);  // lower front rod
    ChVector3d p2R = p1R + ChVector3d(0, 0, 0.2);             // upper front rod
    ChVector3d p5R = rearsusp->GetConnectorLocation(RIGHT);   // lower rear rod
    ChVector3d p4R = p5R + ChVector3d(0, 0, 0.2);             // upper rear rod
    ChVector3d p3R = (p2R + p4R) / 2;                         // beam pivot point

    std::shared_ptr<ChBody> rockerArmL = chrono_types::make_shared<ChBody>();
    rockerArmL->SetName("rockerL");
    rockerArmL->SetPos(p3L);
    rockerArmL->SetRot(m_chassis->GetBody()->GetFrameRefToAbs().GetRot());
    rockerArmL->SetMass(25);
    rockerArmL->SetInertiaXX(ChVector3d(0.05, 0.1, 0.1));
    m_chassis->GetBody()->GetSystem()->AddBody(rockerArmL);

    auto rockerPivL = chrono_types::make_shared<ChLinkLockRevolute>();
    rockerPivL->SetName("rockerPivotL");
    rockerPivL->Initialize(rockerArmL, m_chassis->GetBody(),
                           ChFrame<>(p3L, m_chassis->GetBody()->GetFrameRefToAbs().GetRot() * QuatFromAngleX(CH_PI_2)));
    m_chassis->GetSystem()->AddLink(rockerPivL);

    AddVisualizationLink(rockerArmL, p2L, p4L, 0.03, ChColor(0.8f, 0.2f, 0.2f));
    AddVisualizationLink(rockerArmL, p3L + ChVector3d(0, 0.05, 0), p3L - ChVector3d(0, 0.05, 0), 0.02,
                         ChColor(0.8f, 0.2f, 0.2f));

    std::shared_ptr<ChBody> rockerArmR = chrono_types::make_shared<ChBody>();
    rockerArmR->SetName("rockerR");
    rockerArmR->SetPos(p3R);
    rockerArmR->SetRot(m_chassis->GetBody()->GetFrameRefToAbs().GetRot());
    rockerArmR->SetMass(25);
    rockerArmR->SetInertiaXX(ChVector3d(0.05, 0.1, 0.1));
    m_chassis->GetBody()->GetSystem()->AddBody(rockerArmR);

    auto rockerPivR = chrono_types::make_shared<ChLinkLockRevolute>();
    rockerPivR->SetName("rockerPivotR");
    rockerPivR->Initialize(rockerArmR, m_chassis->GetBody(),
                           ChFrame<>(p3R, m_chassis->GetBody()->GetFrameRefToAbs().GetRot() * QuatFromAngleX(CH_PI_2)));
    m_chassis->GetSystem()->AddLink(rockerPivR);

    AddVisualizationLink(rockerArmR, p2R, p4R, 0.03, ChColor(0.8f, 0.2f, 0.2f));
    AddVisualizationLink(rockerArmR, p3R + ChVector3d(0, 0.05, 0), p3R - ChVector3d(0, 0.05, 0), 0.02,
                         ChColor(0.8f, 0.2f, 0.2f));

    std::shared_ptr<ChBody> frontRodL = chrono_types::make_shared<ChBody>();
    frontRodL->SetName("frontRodL");
    frontRodL->SetPos((p1L + p2L) / 2);
    frontRodL->SetRot(m_chassis->GetBody()->GetFrameRefToAbs().GetRot());
    frontRodL->SetMass(10);
    frontRodL->SetInertiaXX(ChVector3d(0.04, 0.04, 0.01));
    m_chassis->GetBody()->GetSystem()->AddBody(frontRodL);

    AddVisualizationLink(frontRodL, p1L, p2L, 0.03, ChColor(0.8f, 0.8f, 0.2f));

    std::shared_ptr<ChLinkLockSpherical> sphFrontSusL = chrono_types::make_shared<ChLinkLockSpherical>();
    sphFrontSusL->SetName("sphFrontSusL");
    sphFrontSusL->Initialize(frontsusp->GetConnectorBody(), frontRodL, ChFrame<>(p1L, QUNIT));
    m_chassis->GetSystem()->AddLink(sphFrontSusL);

    std::shared_ptr<ChLinkLockSpherical> sphFrontArmL = chrono_types::make_shared<ChLinkLockSpherical>();
    sphFrontArmL->SetName("sphFrontArmL");
    sphFrontArmL->Initialize(rockerArmL, frontRodL, ChFrame<>(p2L, QUNIT));
    m_chassis->GetSystem()->AddLink(sphFrontArmL);

    std::shared_ptr<ChBody> frontRodR = chrono_types::make_shared<ChBody>();
    frontRodR->SetName("frontRodR");
    frontRodR->SetPos((p1R + p2R) / 2);
    frontRodR->SetRot(m_chassis->GetBody()->GetFrameRefToAbs().GetRot());
    frontRodR->SetMass(10);
    frontRodR->SetInertiaXX(ChVector3d(0.04, 0.04, 0.01));
    m_chassis->GetBody()->GetSystem()->AddBody(frontRodR);

    AddVisualizationLink(frontRodR, p1R, p2R, 0.03, ChColor(0.8f, 0.8f, 0.2f));

    std::shared_ptr<ChLinkLockSpherical> sphFrontSusR = chrono_types::make_shared<ChLinkLockSpherical>();
    sphFrontSusR->SetName("sphFrontSusR");
    sphFrontSusR->Initialize(frontsusp->GetConnectorBody(), frontRodR, ChFrame<>(p1R, QUNIT));
    m_chassis->GetSystem()->AddLink(sphFrontSusR);

    std::shared_ptr<ChLinkLockSpherical> sphFrontArmR = chrono_types::make_shared<ChLinkLockSpherical>();
    sphFrontArmR->SetName("sphFrontArmR");
    sphFrontArmR->Initialize(rockerArmR, frontRodR, ChFrame<>(p2R, QUNIT));
    m_chassis->GetSystem()->AddLink(sphFrontArmR);

    std::shared_ptr<ChBody> rearRodL = chrono_types::make_shared<ChBody>();
    rearRodL->SetName("rearRodL");
    rearRodL->SetPos((p4L + p5L) / 2);
    rearRodL->SetRot(m_chassis->GetBody()->GetFrameRefToAbs().GetRot());
    rearRodL->SetMass(10);
    rearRodL->SetInertiaXX(ChVector3d(0.04, 0.04, 0.01));
    m_chassis->GetBody()->GetSystem()->AddBody(rearRodL);

    AddVisualizationLink(rearRodL, p4L, p5L, 0.03, ChColor(0.8f, 0.8f, 0.2f));

    std::shared_ptr<ChBody> rearRodR = chrono_types::make_shared<ChBody>();
    rearRodR->SetName("rearRodR");
    rearRodR->SetPos((p4R + p5R) / 2);
    rearRodR->SetRot(m_chassis->GetBody()->GetFrameRefToAbs().GetRot());
    rearRodR->SetMass(10);
    rearRodR->SetInertiaXX(ChVector3d(0.04, 0.04, 0.01));
    m_chassis->GetBody()->GetSystem()->AddBody(rearRodR);

    AddVisualizationLink(rearRodR, p4R, p5R, 0.03, ChColor(0.8f, 0.8f, 0.2f));

    std::shared_ptr<ChLinkLockSpherical> sphRearSusL = chrono_types::make_shared<ChLinkLockSpherical>();
    sphRearSusL->SetName("sphRearSusL");
    sphRearSusL->Initialize(rearsusp->GetConnectorBody(), rearRodL, ChFrame<>(p5L, QUNIT));
    m_chassis->GetSystem()->AddLink(sphRearSusL);

    std::shared_ptr<ChLinkLockSpherical> sphRearSusR = chrono_types::make_shared<ChLinkLockSpherical>();
    sphRearSusR->SetName("sphRearSusR");
    sphRearSusR->Initialize(rearsusp->GetConnectorBody(), rearRodR, ChFrame<>(p5R, QUNIT));
    m_chassis->GetSystem()->AddLink(sphRearSusR);

    std::shared_ptr<ChLinkLockSpherical> sphRearArmL = chrono_types::make_shared<ChLinkLockSpherical>();
    sphRearArmL->SetName("sphRearArmL");
    sphRearArmL->Initialize(rockerArmL, rearRodL, ChFrame<>(p4L, QUNIT));
    m_chassis->GetSystem()->AddLink(sphRearArmL);

    std::shared_ptr<ChLinkLockSpherical> sphRearArmR = chrono_types::make_shared<ChLinkLockSpherical>();
    sphRearArmR->SetName("sphRearArmR");
    sphRearArmR->Initialize(rockerArmR, rearRodR, ChFrame<>(p4R, QUNIT));
    m_chassis->GetSystem()->AddLink(sphRearArmR);

    /************** end roll stabilizer def ********************************/

    // Initialize the driveline subsystem
    std::vector<int> driven_susp_indexes(m_driveline->GetNumDrivenAxles());
    driven_susp_indexes[0] = 1;
    driven_susp_indexes[1] = 1;
    m_driveline->Initialize(m_chassis, m_axles, driven_susp_indexes);

    // Invoke base class method
    ChWheeledVehicle::Initialize(chassisPos, chassisFwdVel);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Duro_Vehicle::GetSpringForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[axle]->m_suspension)->GetSpringForce(side);
}

double Duro_Vehicle::GetSpringLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[axle]->m_suspension)->GetSpringLength(side);
}

double Duro_Vehicle::GetSpringDeformation(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[axle]->m_suspension)->GetSpringDeformation(side);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double Duro_Vehicle::GetShockForce(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[axle]->m_suspension)->GetShockForce(side);
}

double Duro_Vehicle::GetShockLength(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[axle]->m_suspension)->GetShockLength(side);
}

double Duro_Vehicle::GetShockVelocity(int axle, VehicleSide side) const {
    return std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[axle]->m_suspension)->GetShockVelocity(side);
}

// -----------------------------------------------------------------------------
// Log the hardpoint locations for the front-right and rear-right suspension
// subsystems (display in inches)
// -----------------------------------------------------------------------------
void Duro_Vehicle::LogHardpointLocations() {
    std::cout << "\n---- FRONT suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChToeBarDeDionAxle>(m_axles[0]->m_suspension)
        ->LogHardpointLocations(ChVector3d(0, 0, 0), false);

    std::cout << "\n---- REAR suspension hardpoint locations (LEFT side)\n";
    std::static_pointer_cast<ChDeDionAxle>(m_axles[1]->m_suspension)->LogHardpointLocations(ChVector3d(0, 0, 0), false);

    std::cout << "\n\n";
}

// -----------------------------------------------------------------------------
// Log the spring length, deformation, and force.
// Log the shock length, velocity, and force.
// Log constraint violations of suspension joints.
//
// Lengths are reported in inches, velocities in inches/s, and forces in lbf
// -----------------------------------------------------------------------------
void Duro_Vehicle::DebugLog(int what) {
    if (what & OUT_SPRINGS) {
        std::cout << "\n---- Spring (front-left, front-right, rear-left, rear-right)\n";
        std::cout << "Length [m]       " << GetSpringLength(0, LEFT) << "  " << GetSpringLength(0, RIGHT) << "  "
                  << GetSpringLength(1, LEFT) << "  " << GetSpringLength(1, RIGHT) << "\n";
        std::cout << "Deformation [m]  " << GetSpringDeformation(0, LEFT) << "  " << GetSpringDeformation(0, RIGHT)
                  << "  " << GetSpringDeformation(1, LEFT) << "  " << GetSpringDeformation(1, RIGHT) << "\n";
        std::cout << "Force [N]         " << GetSpringForce(0, LEFT) << "  " << GetSpringForce(0, RIGHT) << "  "
                  << GetSpringForce(1, LEFT) << "  " << GetSpringForce(1, RIGHT) << "\n";
    }

    if (what & OUT_SHOCKS) {
        std::cout << "\n---- Shock (front-left, front-right, rear-left, rear-right)\n";
        std::cout << "Length [m]       " << GetShockLength(0, LEFT) << "  " << GetShockLength(0, RIGHT) << "  "
                  << GetShockLength(1, LEFT) << "  " << GetShockLength(1, RIGHT) << "\n";
        std::cout << "Velocity [m/s]   " << GetShockVelocity(0, LEFT) << "  " << GetShockVelocity(0, RIGHT) << "  "
                  << GetShockVelocity(1, LEFT) << "  " << GetShockVelocity(1, RIGHT) << "\n";
        std::cout << "Force [N]         " << GetShockForce(0, LEFT) << "  " << GetShockForce(0, RIGHT) << "  "
                  << GetShockForce(1, LEFT) << "  " << GetShockForce(1, RIGHT) << "\n";
    }

    if (what & OUT_CONSTRAINTS) {
        // Report constraint violations for all joints
        LogConstraintViolations();
    }
}

// -----------------------------------------------------------------------------
void Duro_Vehicle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                        const ChVector3d pt_1,
                                        const ChVector3d pt_2,
                                        double radius,
                                        const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    cyl->SetColor(color);
}

}  // namespace duro
}  // end namespace vehicle
}  // end namespace chrono
