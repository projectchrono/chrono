// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// NASA Curiosity Mars Rover Model Class.
// This class contains model for NASA's Curiosity Mars rover
//
// =============================================================================

#include <cmath>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_models/robot/curiosity/Curiosity.h"

namespace chrono {
namespace curiosity {

// =============================================================================

// maximum steering angle
static const double max_steer_angle = CH_C_PI / 6;

// rover wheels positions
static const ChVector<> wheel_rel_pos_lf = ChVector<>(1.095, 1.063, 0.249);
static const ChVector<> wheel_rel_pos_rf = ChVector<>(1.095, -1.063, 0.249);
static const ChVector<> wheel_rel_pos_lm = ChVector<>(-0.089, 1.194, 0.249);
static const ChVector<> wheel_rel_pos_rm = ChVector<>(-0.089, -1.194, 0.249);
static const ChVector<> wheel_rel_pos_lb = ChVector<>(-1.163, 1.063, 0.249);
static const ChVector<> wheel_rel_pos_rb = ChVector<>(-1.163, -1.063, 0.249);

// steering rod positions
static const ChVector<> sr_rel_pos_lf = ChVector<>(1.095, 1.063, 0.64);
static const ChVector<> sr_rel_pos_rf = ChVector<>(1.095, -1.063, 0.64);
static const ChVector<> sr_rel_pos_lb = ChVector<>(-1.163, 1.063, 0.64);
static const ChVector<> sr_rel_pos_rb = ChVector<>(-1.163, -1.063, 0.64);

// suspension arm positions
static const ChVector<> cr_rel_pos_lf = ChVector<>(0.214, 0.604, 0.8754);
static const ChVector<> cr_rel_pos_rf = ChVector<>(0.214, -0.604, 0.8754);
static const ChVector<> cr_rel_pos_lb = ChVector<>(-0.54, 0.845, 0.6433);
static const ChVector<> cr_rel_pos_rb = ChVector<>(-0.54, -0.845, 0.6433);

// balancer positions
static const ChVector<> tr_rel_pos_l = ChVector<>(0.214, 0.672, 1.144);
static const ChVector<> tr_rel_pos_r = ChVector<>(0.214, -0.672, 1.144);
static const ChVector<> tr_rel_pos_t = ChVector<>(-0.142, 0.0, 1.172);

// =============================================================================

// Default contact material for rover parts
std::shared_ptr<ChMaterialSurface> DefaultContactMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.0f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddRevoluteJoint(std::shared_ptr<ChBodyAuxRef> body_1,
                      std::shared_ptr<ChBodyAuxRef> body_2,
                      std::shared_ptr<Curiosity_Chassis> chassis,
                      const ChVector<>& rel_joint_pos,
                      const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetBody()->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);                       // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                                       // global -> child

    auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    // auto revo = chrono_types::make_shared<ChLinkLockLock>();
    revo->Initialize(body_1, body_2, ChCoordsys<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    chassis->GetBody()->GetSystem()->AddLink(revo);
}

// Add a rotational speed motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotorSpeed(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<Curiosity_Chassis> chassis,
                                                        const ChVector<>& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational angle motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationAngle> AddMotorAngle(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<Curiosity_Chassis> chassis,
                                                        const ChVector<>& rel_pos,
                                                        const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// Add a rotational torque motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationTorque> AddMotorTorque(std::shared_ptr<ChBody> body1,
                                                          std::shared_ptr<ChBody> body2,
                                                          std::shared_ptr<Curiosity_Chassis> chassis,
                                                          const ChVector<>& rel_pos,
                                                          const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create motor (actuated DOF about Z axis of X_GC frame)
    auto motor = chrono_types::make_shared<ChLinkMotorRotationTorque>();
    motor->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(motor);

    return motor;
}

// ===============================================================================
//
// Base class for all Curiosity Part
Curiosity_Part::Curiosity_Part(const std::string& name,
                               const ChFrame<>& rel_pos,
                               std::shared_ptr<ChMaterialSurface> mat,
                               bool collide)
    : m_name(name), m_pos(rel_pos), m_density(200), m_mat(mat), m_collide(collide), m_visualize(true) {}

void Curiosity_Part::Construct(ChSystem* system) {
    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetNameString(m_name + "_body");

    // Load geometry meshes for visualization and collision
    std::string vis_mesh_file = "robot/curiosity/obj/" + m_mesh_name + ".obj";
    std::string col_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";

    auto trimesh_vis = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh_vis->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
    trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
    trimesh_vis->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

    auto trimesh_col = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh_col->LoadWavefrontMesh(GetChronoDataFile(col_mesh_file), false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

    // Calculate and set intertia properties (use collision mesh)
    double vol;
    ChVector<> cog;
    ChMatrix33<> inertia;
    trimesh_col->ComputeMassProperties(true, vol, cog, inertia);

    ChVector<> principal_I;
    ChMatrix33<> principal_inertia_rot;
    ChInertiaUtils::PrincipalInertia(inertia, principal_I, principal_inertia_rot);

    m_body->SetMass(vol * m_density);
    m_body->SetInertiaXX(m_density * principal_I);
    m_body->SetFrame_COG_to_REF(ChFrame<>(cog, principal_inertia_rot));

    // Add visualization shape
    if (m_visualize) {
        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetStatic(true);
        m_body->AddAsset(trimesh_shape);
        m_body->AddAsset(chrono_types::make_shared<ChColorAsset>(m_color));
    }

    // Add collision shape
    if (m_collide) {
        ////auto trimesh_c = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
        ////trimesh_c->LoadWavefrontMesh(GetChronoDataFile(col_mesh_file), false, false);

        m_body->GetCollisionModel()->ClearModel();
        m_body->GetCollisionModel()->AddTriangleMesh(m_mat, trimesh_col, false, false, VNULL, ChMatrix33<>(1), 0.005);
        m_body->GetCollisionModel()->BuildModel();
        m_body->SetCollide(m_collide);
    }

    system->AddBody(m_body);
}

void Curiosity_Part::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrame_REF_to_abs() * m_pos;
    m_body->SetFrame_REF_to_abs(X_GC);
}

// =============================================================================
//
// Rover Chassis
Curiosity_Chassis::Curiosity_Chassis(const std::string& name,
                                     ChassisType chassis_type,
                                     std::shared_ptr<ChMaterialSurface> mat)
    : Curiosity_Part(name, ChFrame<>(VNULL, QUNIT), mat, false), m_chassis_type(chassis_type) {
    switch (m_chassis_type) {
        case ChassisType::FullRover:
            m_mesh_name = "curiosity_chassis";
            break;

        case ChassisType::Scarecrow:
            m_mesh_name = "scarecrow_chassis";
            break;
    }
    m_density = 50;
    m_color = ChColor(1.0f, 1.0f, 1.0f);
}

void Curiosity_Chassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    double vol = 0;
    ChVector<> inertia;
    switch (m_chassis_type) {
        case ChassisType::FullRover:
            vol = 8.3;
            inertia = ChVector<>(0.2);
            break;
        case ChassisType::Scarecrow:
            vol = 2.3;
            inertia = ChVector<>(0.5);
            break;
    }
    m_body->SetMass(vol * m_density);
    m_body->SetInertiaXX(m_density * inertia);
    m_body->SetFrame_COG_to_REF(ChFrame<>());

    m_body->SetFrame_REF_to_abs(pos);
}

// Curiosity Wheel
Curiosity_Wheel::Curiosity_Wheel(const std::string& name,
                                 const ChFrame<>& rel_pos,
                                 std::shared_ptr<ChMaterialSurface> mat,
                                 WheelType wheel_type)
    : Curiosity_Part(name, rel_pos, mat, true) {
    switch (wheel_type) {
        default:
        case WheelType::RealWheel:
            m_mesh_name = "curiosity_wheel";
            break;
        case WheelType::SimpleWheel:
            m_mesh_name = "curiosity_simplewheel";
            break;
        case WheelType::CylWheel:
            m_mesh_name = "curiosity_cylwheel";
            break;
    }
    m_color = ChColor(1.0f, 1.0f, 1.0f);
    m_density = 2000;
}

void Curiosity_Wheel::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Curiosity_Part::Initialize(chassis);

    double vol = 0.004;
    ChVector<> inertia(0.01);
    m_body->SetMass(vol * m_density);
    m_body->SetInertiaXX(m_density * inertia);
    m_body->SetFrame_COG_to_REF(ChFrame<>());
}

// Curiosity suspension arm
Curiosity_Arm::Curiosity_Arm(const std::string& name,
                             const ChFrame<>& rel_pos,
                             std::shared_ptr<ChMaterialSurface> mat,
                             int which)
    : Curiosity_Part(name, rel_pos, mat, false) {
    switch (which) {
        case 0:
            m_mesh_name = "curiosity_F_L_arm";
            break;
        case 1:
            m_mesh_name = "curiosity_F_R_arm";
            break;
        case 2:
            m_mesh_name = "curiosity_B_L_arm";
            break;
        case 3:
            m_mesh_name = "curiosity_B_R_arm";
            break;
    }
    m_color = ChColor(1.0f, 0.4f, 0.0f);
    m_density = 2000;
}

// Curiosity steering rod
Curiosity_Steer::Curiosity_Steer(const std::string& name,
                                 const ChFrame<>& rel_pos,
                                 std::shared_ptr<ChMaterialSurface> mat)
    : Curiosity_Part(name, rel_pos, mat, false) {
    m_mesh_name = "curiosity_steer";
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 1000;
}

void Curiosity_Steer::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Curiosity_Part::Initialize(chassis);

    double vol = 0.01;
    ChVector<> inertia(0.01);
    m_body->SetMass(vol * m_density);
    m_body->SetInertiaXX(m_density * inertia);
    m_body->SetFrame_COG_to_REF(ChFrame<>());
}

// Curiosity balancers
Curiosity_Balancer::Curiosity_Balancer(const std::string& name,
                                       const ChFrame<>& rel_pos,
                                       std::shared_ptr<ChMaterialSurface> mat,
                                       int which)
    : Curiosity_Part(name, rel_pos, mat, false) {
    switch (which) {
        case 0:
            m_mesh_name = "curiosity_bar_L";
            break;
        case 1:
            m_mesh_name = "curiosity_bar_R";
        case 2:
            m_mesh_name = "curiosity_balancer";
    }
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 1000;
}

// ==========================================================

CuriosityRover::CuriosityRover(ChSystem* system, ChassisType chassis_type, WheelType wheel_type)
    : m_system(system), m_chassis_fixed(false), m_suspension_fixed(false) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == ChContactMethod::NSC) {
        collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    m_default_material = DefaultContactMaterial(contact_method);
    m_wheel_material = DefaultContactMaterial(contact_method);

    Create(chassis_type, wheel_type);
}

void CuriosityRover::Create(ChassisType chassis_type, WheelType wheel_type) {
    // create rover chassis
    ChQuaternion<> body_rot;
    m_chassis = chrono_types::make_shared<Curiosity_Chassis>("chassis", chassis_type, m_default_material);

    // Create 6 Curiosity Rover wheels
    ChVector<> wheel_pos[] = {
        wheel_rel_pos_lf,  // LF
        wheel_rel_pos_rf,  // RF
        wheel_rel_pos_lm,  // LM
        wheel_rel_pos_rm,  // RM
        wheel_rel_pos_lb,  // LB
        wheel_rel_pos_rb   // RB
    };

    for (int i = 0; i < 6; i++) {
        m_wheels[i] = chrono_types::make_shared<Curiosity_Wheel>("wheel", ChFrame<>(wheel_pos[i], QUNIT),
                                                                 m_wheel_material, wheel_type);
    }
    m_wheels[RF]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
    m_wheels[RM]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
    m_wheels[RB]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));

    // Create 4 Curiosity Suspension Arms
    ChVector<> rod_pos[] = {
        cr_rel_pos_lf,  // LF
        cr_rel_pos_rf,  // RF
        cr_rel_pos_lb,  // LB
        cr_rel_pos_rb,  // RB
    };

    for (int i = 0; i < 4; i++) {
        m_arms[i] =
            chrono_types::make_shared<Curiosity_Arm>("arm", ChFrame<>(rod_pos[i], QUNIT), m_default_material, i);
    }

    // Create 4 Curiosity Steering Rods
    ChVector<> steer_pos[] = {
        sr_rel_pos_lf,  // LF
        sr_rel_pos_rf,  // RF
        sr_rel_pos_lb,  // LB
        sr_rel_pos_rb,  // RB
    };

    ChQuaternion<> steer_rot = Q_from_Euler123(ChVector<double>(0, 0, CH_C_PI));
    for (int i = 0; i < 4; i++) {
        if (i == 1 || i == 3) {
            m_steers[i] = chrono_types::make_shared<Curiosity_Steer>("steer", ChFrame<>(steer_pos[i], steer_rot),
                                                                     m_default_material);
        } else {
            m_steers[i] =
                chrono_types::make_shared<Curiosity_Steer>("steer", ChFrame<>(steer_pos[i], QUNIT), m_default_material);
        }
    }

    // Create 3 balancer components
    ChVector<> bal_pos[] = {
        tr_rel_pos_l,  // L
        tr_rel_pos_r,  // R
        tr_rel_pos_t,  // CENTER
    };

    for (int i = 0; i < 3; i++) {
        m_balancers[i] = chrono_types::make_shared<Curiosity_Balancer>("balancer", ChFrame<>(bal_pos[i], QUNIT),
                                                                       m_default_material, i);
    }

    // Create drive shafts
    for (int i = 0; i < 6; i++) {
        m_drive_shafts[i] = chrono_types::make_shared<ChShaft>();
    }
}

void CuriosityRover::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    // Initialize rover parts, fixing bodies to ground as requested
    m_chassis->Initialize(m_system, pos);
    m_chassis->GetBody()->SetBodyFixed(m_chassis_fixed);

    for (int i = 0; i < 6; i++) {
        m_wheels[i]->Initialize(m_chassis->GetBody());
    }
    for (int i = 0; i < 4; i++) {
        m_arms[i]->Initialize(m_chassis->GetBody());
        m_arms[i]->GetBody()->SetBodyFixed(m_suspension_fixed);
    }
    for (int i = 0; i < 4; i++) {
        m_steers[i]->Initialize(m_chassis->GetBody());
        m_steers[i]->GetBody()->SetBodyFixed(m_suspension_fixed);
    }
    for (int i = 0; i < 3; i++) {
        m_balancers[i]->Initialize(m_chassis->GetBody());
    }

    // Add motors on all six wheels
    ChVector<> motor_pos[] = {
        wheel_rel_pos_lf, wheel_rel_pos_rf,  // front (left/right)
        wheel_rel_pos_lm, wheel_rel_pos_rm,  // middle (left/right)
        wheel_rel_pos_lb, wheel_rel_pos_rb   // back (left/right)
    };

    ChQuaternion<> z2y = Q_from_AngX(CH_C_PI_2);  // align Z with (negative) Y

    // Front
    for (int i = 0; i < 2; i++) {
        switch (m_driver->GetDriveMotorType()) {
            case CuriosityDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();
                m_drive_motors[i] =
                    AddMotorSpeed(m_steers[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis, motor_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case CuriosityDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(m_steers[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis, motor_pos[i], z2y);
                break;
        }
    }

    // Middle
    for (int i = 2; i < 4; i++) {
        switch (m_driver->GetDriveMotorType()) {
            case CuriosityDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();
                m_drive_motors[i] =
                    AddMotorSpeed(m_arms[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis, motor_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case CuriosityDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(m_arms[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis, motor_pos[i], z2y);
                break;
        }
    }

    // Rear
    for (int i = 4; i < 6; i++) {
        switch (m_driver->GetDriveMotorType()) {
            case CuriosityDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();
                m_drive_motors[i] =
                    AddMotorSpeed(m_steers[i - 2]->GetBody(), m_wheels[i]->GetBody(), m_chassis, motor_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case CuriosityDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(m_steers[i - 2]->GetBody(), m_wheels[i]->GetBody(), m_chassis, motor_pos[i], z2y);
                break;
        }
    }

    double J = 0.1;  // shaft rotational inertia
    for (int i = 0; i < 6; i++) {
        m_drive_shafts[i]->SetInertia(J);
        m_system->Add(m_drive_shafts[i]);

        // Connect shaft aligned with the wheel's axis of rotation (local wheel Y). 
        // Set connection such that a positive torque applied to the shaft results in forward rover motion.
        auto shaftbody_connection = chrono_types::make_shared<ChShaftsBody>();
        shaftbody_connection->Initialize(m_drive_shafts[i], m_wheels[i]->GetBody(), ChVector<>(0, -1, 0));
        m_system->Add(shaftbody_connection);
    }

    // Add revolute joints between steering rod and suspension arms
    ChVector<> rev_steer_suspension[] = {
        sr_rel_pos_lf, sr_rel_pos_rf,  // front (left/right)
        sr_rel_pos_lb, sr_rel_pos_rb   // back (left/right)
    };

    for (int i = 0; i < 4; i++) {
        m_steer_motor_funcs[i] = chrono_types::make_shared<ChFunction_Const>(0.0);
        m_steer_motors[i] =
            AddMotorAngle(m_steers[i]->GetBody(), m_arms[i]->GetBody(), m_chassis, rev_steer_suspension[i], QUNIT);
        m_steer_motors[i]->SetMotorFunction(m_steer_motor_funcs[i]);
    }

    // Add a revolute joint between the rover body and the top balancer
    AddRevoluteJoint(m_balancers[2]->GetBody(), m_chassis->GetBody(), m_chassis, tr_rel_pos_t, QUNIT);

    // Add revolute joints (1) between top balancer and L/R balancer rods
    //                     (2) between L/R balancer and front suspension arms
    AddRevoluteJoint(m_arms[0]->GetBody(), m_balancers[0]->GetBody(), m_chassis, tr_rel_pos_l, z2y);
    AddRevoluteJoint(m_arms[1]->GetBody(), m_balancers[1]->GetBody(), m_chassis, tr_rel_pos_r, z2y);
    AddRevoluteJoint(m_balancers[0]->GetBody(), m_balancers[2]->GetBody(), m_chassis,
                     tr_rel_pos_l + ChVector<>(tr_rel_pos_t.x() - tr_rel_pos_l.x(), 0, 0), QUNIT);
    AddRevoluteJoint(m_balancers[1]->GetBody(), m_balancers[2]->GetBody(), m_chassis,
                     tr_rel_pos_r + ChVector<>(tr_rel_pos_t.x() - tr_rel_pos_r.x(), 0, 0), QUNIT);

    // Add revolute joint for suspension arms
    AddRevoluteJoint(m_arms[0]->GetBody(), m_chassis->GetBody(), m_chassis, cr_rel_pos_lf, z2y);
    AddRevoluteJoint(m_arms[1]->GetBody(), m_chassis->GetBody(), m_chassis, cr_rel_pos_rf, z2y);
    AddRevoluteJoint(m_arms[2]->GetBody(), m_arms[0]->GetBody(), m_chassis, cr_rel_pos_lb, z2y);
    AddRevoluteJoint(m_arms[3]->GetBody(), m_arms[1]->GetBody(), m_chassis, cr_rel_pos_rb, z2y);
}

void CuriosityRover::SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void CuriosityRover::SetDriver(std::shared_ptr<CuriosityDriver> driver) {
    m_driver = driver;
    m_driver->curiosity = this;
}

void CuriosityRover::SetChassisFixed(bool fixed) {
    m_chassis_fixed = fixed;
}

void CuriosityRover::SetSuspensionFixed(bool fixed) {
    m_suspension_fixed = fixed;
}

void CuriosityRover::SetChassisVisualization(bool state) {
    m_chassis->SetVisualize(state);
}

void CuriosityRover::SetWheelVisualization(bool state) {
    for (auto& wheel : m_wheels)
        wheel->SetVisualize(state);
}

void CuriosityRover::SetSuspensionVisualization(bool state) {
    for (auto& p : m_arms)
        p->SetVisualize(state);
    for (auto& p : m_balancers)
        p->SetVisualize(state);
    for (auto& p : m_steers)
        p->SetVisualize(state);
}

ChVector<> CuriosityRover::GetWheelContactForce(WheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector<> CuriosityRover::GetWheelContactTorque(WheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector<> CuriosityRover::GetWheelAppliedForce(WheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector<> CuriosityRover::GetWheelAppliedTorque(WheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double CuriosityRover::GetWheelTracTorque(WheelID id) const {
    if (m_driver->GetDriveMotorType() == CuriosityDriver::DriveMotorType::TORQUE)
        return 0;

    return m_drive_motors[id]->GetMotorTorque();
}

std::shared_ptr<Curiosity_Wheel> CuriosityRover::GetWheel(WheelID id) const {
    return m_wheels[id];
}

double CuriosityRover::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass();
    for (int i = 0; i < 6; i++) {
        tot_mass = tot_mass + m_wheels[i]->GetBody()->GetMass();
    }
    for (int i = 0; i < 4; i++) {
        tot_mass = tot_mass + m_arms[i]->GetBody()->GetMass();
        tot_mass = tot_mass + m_steers[i]->GetBody()->GetMass();
    }
    for (int i = 0; i < 3; i++) {
        tot_mass = tot_mass + m_balancers[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double CuriosityRover::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

void CuriosityRover::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    if (m_driver->GetDriveMotorType() == CuriosityDriver::DriveMotorType::SPEED) {
        for (int i = 0; i < 6; i++) {
            double driving = m_driver->drive_speeds[i];
            m_drive_motor_funcs[i]->SetSetpoint(driving, time);
        }
    }

    for (int i = 0; i < 4; i++) {
        double steering = m_driver->steer_angles[i];
        // Enforce maximum steering angle
        ChClampValue(steering, -max_steer_angle, +max_steer_angle);
        m_steer_motor_funcs[i]->Set_yconst(steering);
    }
}

// =============================================================================

CuriosityDriver::CuriosityDriver() : drive_speeds({0, 0, 0, 0, 0, 0}), steer_angles({0, 0, 0, 0}), curiosity(nullptr) {}

void CuriosityDriver::SetSteering(double angle) {
    steer_angles = {angle, angle, angle, angle};
}

void CuriosityDriver::SetSteering(double angle, WheelID id) {
    if (id == WheelID::LM || id == WheelID::RM)
        return;
    steer_angles[id] = angle;
}

// -----------------------------------------------------------------------------

CuriosityDCMotorControl::CuriosityDCMotorControl()
    : m_stall_torque({300, 300, 300, 300, 300, 300}),
      m_no_load_speed({CH_C_PI, CH_C_PI, CH_C_PI, CH_C_PI, CH_C_PI, CH_C_PI}) {}

void CuriosityDCMotorControl::Update(double time) {
    double speed_reading;
    double target_torque;
    for (int i = 0; i < 6; i++) {
        speed_reading = -curiosity->m_drive_shafts[i]->GetPos_dt();

        if (speed_reading > m_no_load_speed[i]) {
            target_torque = 0;
        } else if (speed_reading < 0) {
            target_torque = m_stall_torque[i];
        } else {
            target_torque = m_stall_torque[i] * ((m_no_load_speed[i] - speed_reading) / m_no_load_speed[i]);
        }
        curiosity->m_drive_shafts[i]->SetAppliedTorque(-target_torque);
    }
}

CuriositySpeedDriver::CuriositySpeedDriver(double time_ramp, double speed) : m_ramp(time_ramp), m_speed(speed) {}

void CuriositySpeedDriver::Update(double time) {
    double speed = m_speed;
    if (time < m_ramp)
        speed = m_speed * (time / m_ramp);
    drive_speeds = {speed, speed, speed, speed, speed, speed};
}

}  // namespace curiosity
}  // namespace chrono
