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
// Create default contact material for the rover
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
                      std::shared_ptr<ChBodyAuxRef> chassis,
                      ChSystem* system,
                      const ChVector<>& rel_joint_pos,
                      const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);            // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child

    auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    // auto revo = chrono_types::make_shared<ChLinkLockLock>();
    revo->Initialize(body_1, body_2, ChCoordsys<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    system->AddLink(revo);
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddLockJoint(std::shared_ptr<ChBodyAuxRef> body_1,
                  std::shared_ptr<ChBodyAuxRef> body_2,
                  std::shared_ptr<ChBodyAuxRef> chassis,
                  ChSystem* system,
                  const ChVector<>& rel_joint_pos,
                  const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);            // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child

    // auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    auto revo = chrono_types::make_shared<ChLinkLockLock>();
    revo->Initialize(body_1, body_2, ChCoordsys<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    system->AddLink(revo);
}

// Add a rotational speed controlled motor between body 'body_A' and body 'body_B'
// rel_joint_pos and rel_joint_rot are the position and the rotation of the motor
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotor(std::shared_ptr<ChBodyAuxRef> body_A,
                                                   std::shared_ptr<ChBodyAuxRef> body_B,
                                                   std::shared_ptr<ChBodyAuxRef> chassis,
                                                   ChSystem* system,
                                                   const ChVector<>& rel_joint_pos,
                                                   const ChQuaternion<>& rel_joint_rot,
                                                   std::shared_ptr<ChFunction_Const> speed_func) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);            // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child

    auto motor_angle = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_angle->Initialize(body_A, body_B, ChFrame<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    system->AddLink(motor_angle);
    motor_angle->SetSpeedFunction(speed_func);
    return motor_angle;
}

// Add a spring between pos_1 and pos_2
// the default length of the spring is auto-adjusted
std::shared_ptr<ChLinkTSDA> AddSuspensionSpring(std::shared_ptr<ChBodyAuxRef> chassis,
                                                std::shared_ptr<ChBodyAuxRef> steer,
                                                ChSystem* system,
                                                const ChVector<>& pos_1,
                                                const ChVector<>& pos_2) {
    ChQuaternion<> ori = ChQuaternion<>(1, 0, 0, 0);
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent

    ChFrame<> X_PC_1(pos_1, ori);      // parent -> child
    ChFrame<> X_GC_1 = X_GP * X_PC_1;  // global -> child

    ChFrame<> X_PC_2(pos_2, ori);      // parent -> child
    ChFrame<> X_GC_2 = X_GP * X_PC_2;  // global -> child

    std::shared_ptr<ChLinkTSDA> spring;
    spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(chassis, steer, false, X_GC_1.GetCoord().pos, X_GC_2.GetCoord().pos, true, 0.0);
    spring->SetSpringCoefficient(800000.0);
    spring->SetDampingCoefficient(10000.0);
    system->AddLink(spring);
    return spring;
}

// ===============================================================================
// Base class for all Curiosity Part
Curiosity_Part::Curiosity_Part(const std::string& name,
                               bool fixed,
                               std::shared_ptr<ChMaterialSurface> mat,
                               ChSystem* system,
                               const ChVector<>& body_pos,
                               const ChQuaternion<>& body_rot,
                               std::shared_ptr<ChBodyAuxRef> chassis_body,
                               bool collide) {
    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetNameString(name + "_body");
    m_chassis = chassis_body;
    m_mat = mat;
    m_pos = body_pos;
    m_rot = body_rot;
    m_system = system;
    m_collide = collide;
    m_fixed = fixed;
}

// Create Visulization assets -> Finer mesh
void Curiosity_Part::AddVisualizationAssets() {
    // Read mesh from the obj folder
    std::string vis_mesh_file = "robot/curiosity/vis/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), true, false);
    trimesh->Transform(m_offset, ChMatrix33<>(1));
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_mesh_name);
    trimesh_shape->SetStatic(true);
    m_body->AddAsset(trimesh_shape);
    return;
}

void Curiosity_Part::SetCollide(bool state) {
    m_collide = state;
}

// Add collision assets -> Rougher mesh
void Curiosity_Part::AddCollisionShapes() {
    // read mesh from the col folder
    std::string vis_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    m_trimesh = trimesh;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), true, false);
    trimesh->Transform(m_offset, ChMatrix33<>(1));
    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_mesh_name);
    trimesh_shape->SetStatic(true);

    m_body->GetCollisionModel()->ClearModel();
    m_body->GetCollisionModel()->AddTriangleMesh(m_mat, trimesh, false, false, VNULL, ChMatrix33<>(1), 0.005);
    m_body->GetCollisionModel()->BuildModel();
    m_body->SetCollide(m_collide);
}

// =============================================================================
// Rover Chassis
Curiosity_Chassis::Curiosity_Chassis(const std::string& name,
                                     bool fixed,
                                     std::shared_ptr<ChMaterialSurface> mat,
                                     ChSystem* system,
                                     const ChVector<>& body_pos,
                                     const ChQuaternion<>& body_rot,
                                     bool collide,
                                     Chassis_Type chassis_type)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, NULL, collide) {
    m_chassis_type = chassis_type;
    switch (m_chassis_type) {
        case Chassis_Type::FullRover:
            m_mesh_name = "curiosity_chassis";
            break;

        case Chassis_Type::Scarecrow:
            m_mesh_name = "scarecrow_chassis";
            break;

        default:
            m_mesh_name = "curiosity_chassis";
            break;
    }

    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Curiosity_Chassis::Initialize() {
    std::string vis_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    switch (m_chassis_type) {
        case Chassis_Type::FullRover:
            mmass = 4.3;
            mcog = ChVector<>(0, 0, 0);
            minertia = ChMatrix33<>(2.0);
            break;

        case Chassis_Type::Scarecrow:
            mmass = 1.0;
            mcog = ChVector<>(0, 0, 0);
            minertia = ChMatrix33<>(0.5);
            break;

        default:
            mmass = 4.3;
            mcog = ChVector<>(0, 0, 0);
            minertia = ChMatrix33<>(2.0);
            break;
    }

    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);
    m_body->SetFrame_REF_to_abs(ChFrame<>(m_pos, m_rot));
    m_body->SetBodyFixed(m_fixed);

    AddCollisionShapes();
    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Curiosity_Chassis::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Curiosity_Chassis::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Curiosity Wheel
Curiosity_Wheel::Curiosity_Wheel(const std::string& name,
                                 bool fixed,
                                 std::shared_ptr<ChMaterialSurface> mat,
                                 ChSystem* system,
                                 const ChVector<>& body_pos,
                                 const ChQuaternion<>& body_rot,
                                 std::shared_ptr<ChBodyAuxRef> chassis,
                                 bool collide,
                                 Wheel_Type wheel_type)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    switch (wheel_type) {
        case Wheel_Type::RealWheel:
            m_mesh_name = "curiosity_wheel";
            break;

        case Wheel_Type::SimpleWheel:
            m_mesh_name = "curiosity_simplewheel";
            break;

        case Wheel_Type::CylWheel:
            m_mesh_name = "curiosity_cylwheel";
            break;

        default:
            m_mesh_name = "curiosity_wheel";
            break;
    }

    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void Curiosity_Wheel::Initialize() {
    std::string vis_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    mmass = 0.1;
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                              // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                              // global -> child
    m_body->SetFrame_REF_to_abs(X_GC);
    m_body->SetBodyFixed(m_fixed);

    AddCollisionShapes();
    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Curiosity_Wheel::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Curiosity_Wheel::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Curiosity suspension arm
Curiosity_Arm::Curiosity_Arm(const std::string& name,
                             bool fixed,
                             std::shared_ptr<ChMaterialSurface> mat,
                             ChSystem* system,
                             const ChVector<>& body_pos,
                             const ChQuaternion<>& body_rot,
                             std::shared_ptr<ChBodyAuxRef> chassis,
                             bool collide,
                             const int& side)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    if (side == 0) {
        m_mesh_name = "curiosity_F_L_arm";
    }
    if (side == 1) {
        m_mesh_name = "curiosity_F_R_arm";
    }
    if (side == 2) {
        m_mesh_name = "curiosity_B_L_arm";
    }
    if (side == 3) {
        m_mesh_name = "curiosity_B_R_arm";
    }

    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 5;
}

void Curiosity_Arm::Initialize() {
    std::string vis_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    mmass = 4.3;
    mcog = ChVector<>(0, 0, 0);
    minertia = ChMatrix33<>(5.0);

    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                              // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                              // global -> child
    m_body->SetFrame_REF_to_abs(X_GC);
    m_body->SetBodyFixed(m_fixed);

    AddCollisionShapes();
    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Curiosity_Arm::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Curiosity_Arm::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Curiosity steering rod
Curiosity_Steer::Curiosity_Steer(const std::string& name,
                                 bool fixed,
                                 std::shared_ptr<ChMaterialSurface> mat,
                                 ChSystem* system,
                                 const ChVector<>& body_pos,
                                 const ChQuaternion<>& body_rot,
                                 std::shared_ptr<ChBodyAuxRef> chassis,
                                 bool collide)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "curiosity_steer";

    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Curiosity_Steer::Initialize() {
    std::string vis_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                              // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                              // global -> child
    m_body->SetFrame_REF_to_abs(X_GC);
    m_body->SetBodyFixed(m_fixed);

    AddCollisionShapes();
    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Curiosity_Steer::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Curiosity_Steer::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Curiosity balancers
Curiosity_Balancer::Curiosity_Balancer(const std::string& name,
                                       bool fixed,
                                       std::shared_ptr<ChMaterialSurface> mat,
                                       ChSystem* system,
                                       const ChVector<>& body_pos,
                                       const ChQuaternion<>& body_rot,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       bool collide,
                                       const int& side)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    if (side == 0) {
        m_mesh_name = "curiosity_bar_L";
    }
    if (side == 1) {
        m_mesh_name = "curiosity_bar_R";
    }
    if (side == 2) {
        m_mesh_name = "curiosity_balancer";
    }
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void Curiosity_Balancer::Initialize() {
    std::string vis_mesh_file = "robot/curiosity/col/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
    trimesh->LoadWavefrontMesh(GetChronoDataFile(vis_mesh_file), false, false);
    trimesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector<> mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector<> principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                              // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                              // global -> child
    m_body->SetFrame_REF_to_abs(X_GC);
    m_body->SetBodyFixed(m_fixed);

    AddCollisionShapes();
    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Curiosity_Balancer::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Curiosity_Balancer::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Rover Class for the entire rover model
CuriosityRover::CuriosityRover(ChSystem* system,
                               const ChVector<>& rover_pos,
                               const ChQuaternion<>& rover_rot,
                               std::shared_ptr<ChMaterialSurface> wheel_mat,
                               Chassis_Type chassis_type,
                               Wheel_Type wheel_type)
    : m_system(system),
      m_rover_pos(rover_pos),
      m_rover_rot(rover_rot),
      m_wheel_material(wheel_mat),
      m_custom_wheel_mat(true),
      m_chassis_type(chassis_type),
      m_wheel_type(wheel_type) {
    Create();
}

CuriosityRover::CuriosityRover(ChSystem* system,
                               const ChVector<>& rover_pos,
                               const ChQuaternion<>& rover_rot,
                               Chassis_Type chassis_type,
                               Wheel_Type wheel_type)
    : m_system(system),
      m_rover_pos(rover_pos),
      m_rover_rot(rover_rot),
      m_custom_wheel_mat(false),
      m_chassis_type(chassis_type),
      m_wheel_type(wheel_type) {
    Create();
}

CuriosityRover::~CuriosityRover() {}

void CuriosityRover::Create() {
    auto contact_method = m_system->GetContactMethod();

    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    if (contact_method == ChContactMethod::NSC) {
        collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    if (m_custom_wheel_mat == true) {
        m_chassis_material = DefaultContactMaterial(contact_method);
        m_arm_material = DefaultContactMaterial(contact_method);
        m_steer_material = DefaultContactMaterial(contact_method);
        m_balancer_material = DefaultContactMaterial(contact_method);
    } else {
        // Create the contact materials (all with default properties)
        m_chassis_material = DefaultContactMaterial(contact_method);
        m_wheel_material = DefaultContactMaterial(contact_method);
        m_arm_material = DefaultContactMaterial(contact_method);
        m_steer_material = DefaultContactMaterial(contact_method);
        m_balancer_material = DefaultContactMaterial(contact_method);
    }

    // initialize rover chassis
    ChQuaternion<> body_rot;
    // TEMP TODO: move this body
    m_chassis = chrono_types::make_shared<Curiosity_Chassis>("chassis", false, m_chassis_material, m_system,
                                                             m_rover_pos, m_rover_rot, false, m_chassis_type);
    // initilize rover wheels
    ChVector<> wheel_rel_pos_lf = ChVector<>(1.095, 1.063, 0.249);
    ChVector<> wheel_rel_pos_rf = ChVector<>(1.095, -1.063, 0.249);
    ChVector<> wheel_rel_pos_lm = ChVector<>(-0.089, 1.194, 0.249);
    ChVector<> wheel_rel_pos_rm = ChVector<>(-0.089, -1.194, 0.249);
    ChVector<> wheel_rel_pos_lb = ChVector<>(-1.163, 1.063, 0.249);
    ChVector<> wheel_rel_pos_rb = ChVector<>(-1.163, -1.063, 0.249);

    {
        ChVector<> wheel_pos;
        ChQuaternion<> wheel_rot_ori = ChQuaternion<>(1, 0, 0, 0);
        ChQuaternion<> wheel_rot_ori_pi;
        wheel_rot_ori_pi = Q_from_Euler123(ChVector<double>(0, 0, CH_C_PI));

        for (int i = 0; i < 6; i++) {
            if (i == 0) {
                wheel_pos = wheel_rel_pos_lf;
            }
            if (i == 1) {
                wheel_pos = wheel_rel_pos_rf;
            }
            if (i == 2) {
                wheel_pos = wheel_rel_pos_lm;
            }
            if (i == 3) {
                wheel_pos = wheel_rel_pos_rm;
            }
            if (i == 4) {
                wheel_pos = wheel_rel_pos_lb;
            }
            if (i == 5) {
                wheel_pos = wheel_rel_pos_rb;
            }
            std::shared_ptr<Curiosity_Wheel> m_wheel;
            if (i == 1 || i == 3 || i == 5) {
                m_wheel = chrono_types::make_shared<Curiosity_Wheel>("wheel", false, m_wheel_material, m_system,
                                                                     wheel_pos, wheel_rot_ori_pi, m_chassis->GetBody(),
                                                                     true, m_wheel_type);
            } else {
                m_wheel =
                    chrono_types::make_shared<Curiosity_Wheel>("wheel", false, m_wheel_material, m_system, wheel_pos,
                                                               wheel_rot_ori, m_chassis->GetBody(), true, m_wheel_type);
            }
            m_wheels.push_back(m_wheel);
        }
    }

    // connecting arm
    ChVector<> cr_rel_pos_lf = ChVector<>(0.214, 0.604, 0.8754);
    ChVector<> cr_rel_pos_rf = ChVector<>(0.214, -0.604, 0.8754);
    ChVector<> cr_rel_pos_lb = ChVector<>(-0.54, 0.845, 0.6433);
    ChVector<> cr_rel_pos_rb = ChVector<>(-0.54, -0.845, 0.6433);
    {
        ChVector<> rod_pos;
        ChQuaternion<> rod_ori = ChQuaternion<>(1, 0, 0, 0);

        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                rod_pos = cr_rel_pos_lf;
            }
            if (i == 1) {
                rod_pos = cr_rel_pos_rf;
            }
            if (i == 2) {
                rod_pos = cr_rel_pos_lb;
            }
            if (i == 3) {
                rod_pos = cr_rel_pos_rb;
            }
            std::shared_ptr<Curiosity_Arm> m_arm;

            m_arm = chrono_types::make_shared<Curiosity_Arm>("arm", false, m_arm_material, m_system, rod_pos, rod_ori,
                                                             m_chassis->GetBody(), false, i);

            m_arms.push_back(m_arm);
        }
    }

    ChVector<> sr_rel_pos_lf = ChVector<>(1.095, 1.063, 0.64);
    ChVector<> sr_rel_pos_rf = ChVector<>(1.095, -1.063, 0.64);
    ChVector<> sr_rel_pos_lb = ChVector<>(-1.163, 1.063, 0.64);
    ChVector<> sr_rel_pos_rb = ChVector<>(-1.163, -1.063, 0.64);
    {
        ChVector<> steer_pos;
        ChQuaternion<> steer_ori = ChQuaternion<>(1, 0, 0, 0);
        ChQuaternion<> steer_ori_pi = Q_from_Euler123(ChVector<double>(0, 0, CH_C_PI));
        for (int i = 0; i < 4; i++) {
            if (i == 0) {
                steer_pos = sr_rel_pos_lf;
            }
            if (i == 1) {
                steer_pos = sr_rel_pos_rf;
            }
            if (i == 2) {
                steer_pos = sr_rel_pos_lb;
            }
            if (i == 3) {
                steer_pos = sr_rel_pos_rb;
            }
            std::shared_ptr<Curiosity_Steer> m_steer;

            if (i == 1 || i == 3) {
                m_steer = chrono_types::make_shared<Curiosity_Steer>(
                    "steer", false, m_steer_material, m_system, steer_pos, steer_ori_pi, m_chassis->GetBody(), false);
            } else {
                m_steer = chrono_types::make_shared<Curiosity_Steer>("steer", false, m_steer_material, m_system,
                                                                     steer_pos, steer_ori, m_chassis->GetBody(), false);
            }

            m_steers.push_back(m_steer);
        }
    }

    ChVector<> tr_rel_pos_l = ChVector<>(0.214, 0.672, 1.144);
    ChVector<> tr_rel_pos_r = ChVector<>(0.214, -0.672, 1.144);
    ChVector<> tr_rel_pos_t = ChVector<>(-0.142, 0.0, 1.172);
    {
        ChVector<> bal_pos;
        ChQuaternion<> bal_ori = ChQuaternion<>(1, 0, 0, 0);
        for (int i = 0; i < 3; i++) {
            if (i == 0) {
                bal_pos = tr_rel_pos_l;
            }
            if (i == 1) {
                bal_pos = tr_rel_pos_r;
            }
            if (i == 2) {
                bal_pos = tr_rel_pos_t;
            }
            std::shared_ptr<Curiosity_Balancer> m_balancer;

            m_balancer = chrono_types::make_shared<Curiosity_Balancer>(
                "balancer", false, m_balancer_material, m_system, bal_pos, bal_ori, m_chassis->GetBody(), false, i);

            m_balancers.push_back(m_balancer);
        }
    }

    // DC Motor Test
    for (int i = 0; i < 6; i++) {
        m_power_shafts.push_back(chrono_types::make_shared<ChShaft>());
        m_driven_shafts.push_back(chrono_types::make_shared<ChShaft>());
        m_shaft_gears.push_back(chrono_types::make_shared<ChShaftsGear>());
    }
}

void CuriosityRover::Initialize() {
    m_chassis->Initialize();
    for (int i = 0; i < 6; i++) {
        m_wheels[i]->Initialize();
    }
    for (int i = 0; i < 4; i++) {
        m_arms[i]->Initialize();
    }
    for (int i = 0; i < 4; i++) {
        m_steers[i]->Initialize();
    }
    for (int i = 0; i < 3; i++) {
        m_balancers[i]->Initialize();
    }

    // redeclare position data
    ChVector<> wheel_rel_pos_lf = ChVector<>(1.095, 1.063, 0.249);
    ChVector<> wheel_rel_pos_rf = ChVector<>(1.095, -1.063, 0.249);
    ChVector<> wheel_rel_pos_lm = ChVector<>(-0.089, 1.194, 0.249);
    ChVector<> wheel_rel_pos_rm = ChVector<>(-0.089, -1.194, 0.249);
    ChVector<> wheel_rel_pos_lb = ChVector<>(-1.163, 1.063, 0.249);
    ChVector<> wheel_rel_pos_rb = ChVector<>(-1.163, -1.063, 0.249);

    ChVector<> cr_rel_pos_lf = ChVector<>(0.214, 0.604, 0.8754);
    ChVector<> cr_rel_pos_rf = ChVector<>(0.214, -0.604, 0.8754);
    ChVector<> cr_rel_pos_lb = ChVector<>(-0.54, 0.845, 0.6433);
    ChVector<> cr_rel_pos_rb = ChVector<>(-0.54, -0.845, 0.6433);

    ChVector<> sr_rel_pos_lf = ChVector<>(1.095, 1.063, 0.64);
    ChVector<> sr_rel_pos_rf = ChVector<>(1.095, -1.063, 0.64);
    ChVector<> sr_rel_pos_lb = ChVector<>(-1.163, 1.063, 0.64);
    ChVector<> sr_rel_pos_rb = ChVector<>(-1.163, -1.063, 0.64);

    ChVector<> tr_rel_pos_l = ChVector<>(0.214, 0.672, 1.144);
    ChVector<> tr_rel_pos_r = ChVector<>(0.214, -0.672, 1.144);
    ChVector<> tr_rel_pos_t = ChVector<>(-0.142, 0.0, 1.172);

    ChQuaternion<> ori = ChQuaternion<>(1, 0, 0, 0);

    // Add motors on all six wheels
    for (int i = 0; i < 6; i++) {
        ChVector<> motor_pos;
        if (i == 0) {
            motor_pos = wheel_rel_pos_lf;
        }
        if (i == 1) {
            motor_pos = wheel_rel_pos_rf;
        }
        if (i == 2) {
            motor_pos = wheel_rel_pos_lm;
        }
        if (i == 3) {
            motor_pos = wheel_rel_pos_rm;
        }
        if (i == 4) {
            motor_pos = wheel_rel_pos_lb;
        }
        if (i == 5) {
            motor_pos = wheel_rel_pos_rb;
        }

        ChQuaternion<> z2y = Q_from_AngAxis(CH_C_PI / 2, ChVector<>(1, 0, 0));
        auto const_speed_function = chrono_types::make_shared<ChFunction_Const>(CH_C_PI);  // speed w=3.145 rad/sec
        if (i == 2 || i == 3) {
            if (m_dc_motor_control == false) {
                m_motors_func.push_back(const_speed_function);
                m_motors.push_back(AddMotor(m_arms[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis->GetBody(),
                                            m_system, motor_pos, z2y, const_speed_function));
            } else {
                AddRevoluteJoint(m_arms[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis->GetBody(), m_system,
                                 motor_pos, z2y);
            }

        } else if (i == 0 || i == 1) {
            if (m_dc_motor_control == false) {
                m_motors_func.push_back(const_speed_function);
                m_motors.push_back(AddMotor(m_steers[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis->GetBody(),
                                            m_system, motor_pos, z2y, const_speed_function));
            } else {
                AddRevoluteJoint(m_steers[i]->GetBody(), m_wheels[i]->GetBody(), m_chassis->GetBody(), m_system,
                                 motor_pos, z2y);
            }

        } else {
            if (m_dc_motor_control == false) {
                m_motors_func.push_back(const_speed_function);
                m_motors.push_back(AddMotor(m_steers[i - 2]->GetBody(), m_wheels[i]->GetBody(), m_chassis->GetBody(),
                                            m_system, motor_pos, z2y, const_speed_function));
            } else {
                AddRevoluteJoint(m_steers[i - 2]->GetBody(), m_wheels[i]->GetBody(), m_chassis->GetBody(), m_system,
                                 motor_pos, z2y);
            }
        }
    }

    if (m_dc_motor_control == true) {
        // DC Motor Test
        double J1 = 10;   // inertia of first shaft
        double J2 = 100;  // inertia of second shaft
        double r = -1;    // gear transmission ratio
        double T = 500;   // torque applied to first shaft
        for (int i = 0; i < 6; i++) {
            m_power_shafts[i]->SetInertia(J1);
            if (i == 0 || i == 2 || i == 4) {
                m_power_shafts[i]->SetAppliedTorque(T);
            } else {
                m_power_shafts[i]->SetAppliedTorque(-T);
            }

            m_system->Add(m_power_shafts[i]);

            m_driven_shafts[i]->SetInertia(J2);
            m_system->Add(m_driven_shafts[i]);

            m_shaft_gears[i]->Initialize(m_power_shafts[i], m_driven_shafts[i]);
            m_shaft_gears[i]->SetTransmissionRatio(r);
            m_system->Add(m_shaft_gears[i]);

            auto shaftbody_connection = chrono_types::make_shared<ChShaftsBody>();
            ChVector<> shaftdir(VECT_Z);
            shaftbody_connection->Initialize(m_driven_shafts[i], m_wheels[i]->GetBody(), shaftdir);
            m_system->Add(shaftbody_connection);
        }
    }

    // Add revolute joints between steering rod and suspension arms
    ChVector<> rev_steer_suspension;
    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            rev_steer_suspension = sr_rel_pos_lf;
        }
        if (i == 1) {
            rev_steer_suspension = sr_rel_pos_rf;
        }
        if (i == 2) {
            rev_steer_suspension = sr_rel_pos_lb;
        }
        if (i == 3) {
            rev_steer_suspension = sr_rel_pos_rb;
        }
        // set up steering motors
        // defaultly speed to 0
        auto const_steer_function = chrono_types::make_shared<ChFunction_Const>(0);  // speed w=3.145 rad/sec

        m_steer_motors.push_back(AddMotor(m_steers[i]->GetBody(), m_arms[i]->GetBody(), m_chassis->GetBody(), m_system,
                                          rev_steer_suspension, ori, const_steer_function));
        m_steer_motors_func.push_back(const_steer_function);
    }

    ChVector<> rev_balancer_body = tr_rel_pos_t;
    ChQuaternion<> balancer_body_rot = ChQuaternion<>(1, 0, 0, 0);
    // Add a revolute joint between the rover body and the top balancer
    AddRevoluteJoint(m_balancers[2]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system, rev_balancer_body,
                     balancer_body_rot);

    // Add revolute joints (1) between top balancer and L/R balancer rods
    //                     (2) between L/R balancer and front suspension arms
    ChVector<> balancer_pos = ChVector<>(0, 0, 0);
    ChQuaternion<> balancer_rot = ChQuaternion<>(1, 0, 0, 0);
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            if (i == 0 && j == 0) {
                balancer_pos = tr_rel_pos_l;
                balancer_rot = Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
                AddRevoluteJoint(m_arms[0]->GetBody(), m_balancers[0]->GetBody(), m_chassis->GetBody(), m_system,
                                 balancer_pos, balancer_rot);
            }

            if (i == 0 && j == 1) {
                balancer_pos = tr_rel_pos_l + ChVector<>(tr_rel_pos_t.x() - tr_rel_pos_l.x(), 0, 0);
                balancer_rot = ChQuaternion<>(1, 0, 0, 0);
                AddRevoluteJoint(m_balancers[0]->GetBody(), m_balancers[2]->GetBody(), m_chassis->GetBody(), m_system,
                                 balancer_pos, balancer_rot);
            }

            if (i == 1 && j == 0) {
                balancer_pos = tr_rel_pos_r;
                balancer_rot = Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
                AddRevoluteJoint(m_arms[1]->GetBody(), m_balancers[1]->GetBody(), m_chassis->GetBody(), m_system,
                                 balancer_pos, balancer_rot);
            }

            if (i == 1 && j == 1) {
                balancer_pos = tr_rel_pos_r + ChVector<>(tr_rel_pos_t.x() - tr_rel_pos_r.x(), 0, 0);
                balancer_rot = ChQuaternion<>(1, 0, 0, 0);
                AddRevoluteJoint(m_balancers[1]->GetBody(), m_balancers[2]->GetBody(), m_chassis->GetBody(), m_system,
                                 balancer_pos, balancer_rot);
            }
        }
    }

    // Add revolute joint for suspension arms
    for (int i = 0; i < 4; i++) {
        if (i == 0) {
            ChVector<> rev_pos = cr_rel_pos_lf;
            ChQuaternion<> rev_rot = Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
            AddRevoluteJoint(m_arms[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system, rev_pos,
                             rev_rot);
        }

        if (i == 1) {
            ChVector<> rev_pos = cr_rel_pos_rf;
            ChQuaternion<> rev_rot = Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
            AddRevoluteJoint(m_arms[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system, rev_pos,
                             rev_rot);
        }

        if (i == 2) {
            ChVector<> rev_pos = cr_rel_pos_lb;
            ChQuaternion<> rev_rot = Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
            AddRevoluteJoint(m_arms[i]->GetBody(), m_arms[i - 2]->GetBody(), m_chassis->GetBody(), m_system, rev_pos,
                             rev_rot);
        }

        if (i == 3) {
            ChVector<> rev_pos = cr_rel_pos_rb;
            ChQuaternion<> rev_rot = Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
            AddRevoluteJoint(m_arms[i]->GetBody(), m_arms[i - 2]->GetBody(), m_chassis->GetBody(), m_system, rev_pos,
                             rev_rot);
        }
    }
}

/// Get Chassis Type
Chassis_Type CuriosityRover::GetChassisType() {
    return m_chassis_type;
}

void CuriosityRover::SetMotorSpeed(double rad_speed, WheelID id) {
    m_motors_func[id]->Set_yconst(rad_speed);
    if (m_dc_motor_control == true) {
        m_no_load_speed[id] = rad_speed;
    }
}

void CuriosityRover::SetMotorStallTorque(double torque, WheelID id) {
    if (m_dc_motor_control == true) {
        m_stall_torque[id] = torque;
    } else {
        std::cout << "DC_Control set to false, torque set invalid" << std::endl;
    }
}

void CuriosityRover::SetDCControl(bool dc_control) {
    m_dc_motor_control = dc_control;
    if (m_dc_motor_control == true) {
        for (int i = 0; i < 6; i++) {
            m_stall_torque.push_back(500);
            m_no_load_speed.push_back(CH_C_PI);
        }
    }
}

void CuriosityRover::SetSteerSpeed(double speed, WheelID id) {
    if (abs(speed) > CH_C_PI) {
        std::cout << "invalid steering speed, max is w = pi rad/s" << std::endl;
        return;
    }
    if (id == WheelID::LM || id == WheelID::RM) {
        std::cout << "Middle wheels don't have steering capability! Invalid SetSpeed() call" << std::endl;
        return;
    }

    switch (id) {
        case WheelID::LF:
            return m_steer_motors_func[0]->Set_yconst(speed);
            break;

        case WheelID::RF:
            return m_steer_motors_func[1]->Set_yconst(speed);
            break;

        case WheelID::LB:
            return m_steer_motors_func[2]->Set_yconst(speed);
            break;

        case WheelID::RB:
            return m_steer_motors_func[3]->Set_yconst(speed);
            break;

        default:
            break;
    }
}

ChVector<> CuriosityRover::GetChassisVel() {
    return m_chassis->GetBody()->GetPos_dt();
}

ChVector<> CuriosityRover::GetChassisAcc() {
    return m_chassis->GetBody()->GetPos_dtdt();
}

ChQuaternion<> CuriosityRover::GetChassisRot() {
    return m_chassis->GetBody()->GetRot();
}

ChVector<> CuriosityRover::GetWheelSpeed(WheelID id) {
    return m_wheels[id]->GetBody()->GetPos_dt();
}

ChVector<> CuriosityRover::GetWheelAngVel(WheelID id) {
    return m_wheels[id]->GetBody()->GetWvel_par();
}

ChVector<> CuriosityRover::GetWheelContactForce(WheelID id) {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector<> CuriosityRover::GetWheelContactTorque(WheelID id) {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector<> CuriosityRover::GetWheelAppliedForce(WheelID id) {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector<> CuriosityRover::GetWheelAppliedTorque(WheelID id) {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double CuriosityRover::GetWheelTracTorque(WheelID id) {
    if (m_dc_motor_control == false) {
        return m_motors[id]->GetMotorTorque();
    } else {
        return 0.0;
    }
}

double CuriosityRover::GetDCPowerShaftTorque(WheelID id) {
    if (!m_dc_motor_control) {
        std::cout << "DC motor control is set to false::Invalid GetDCPowerShaftTorque() call" << std::endl;
        return -1;
    }
    return m_power_shafts[id]->GetAppliedTorque();
}

std::shared_ptr<ChBodyAuxRef> CuriosityRover::GetWheelBody(WheelID id) {
    return m_wheels[id]->GetBody();
}

std::shared_ptr<geometry::ChTriangleMeshConnected> CuriosityRover::GetWheelTrimesh(WheelID id) {
    return m_wheels[id]->GetTrimesh();
}

std::shared_ptr<Curiosity_Wheel> CuriosityRover::GetWheelPart(WheelID id) {
    return m_wheels[id];
}

std::shared_ptr<ChBodyAuxRef> CuriosityRover::GetChassisBody() {
    return m_chassis->GetBody();
}

double CuriosityRover::GetSteerAngle(WheelID id) {
    if (id == WheelID::LM || id == WheelID::RM) {
        std::cout << "Middle wheels don't have steering capability! Invalid GetAngle() call" << std::endl;
        return 4 * CH_C_PI;
    }

    switch (id) {
        case WheelID::LF:
            return m_steer_motors[0]->GetMotorRot();
            break;

        case WheelID::RF:
            return m_steer_motors[1]->GetMotorRot();
            break;

        case WheelID::LB:
            return m_steer_motors[2]->GetMotorRot();
            break;

        case WheelID::RB:
            return m_steer_motors[3]->GetMotorRot();
            break;

        default:
            break;
    }

    return 4 * CH_C_PI;
}

double CuriosityRover::GetSteerSpeed(WheelID id) {
    if (id == WheelID::LM || id == WheelID::RM) {
        std::cout << "Middle wheels don't have steering capability! Invalid GetSpeed() call" << std::endl;
        return 4 * CH_C_PI;
    }

    switch (id) {
        case WheelID::LF:
            return m_steer_motors_func[0]->Get_yconst();
            break;

        case WheelID::RF:
            return m_steer_motors_func[1]->Get_yconst();
            break;

        case WheelID::LB:
            return m_steer_motors_func[2]->Get_yconst();
            break;

        case WheelID::RB:
            return m_steer_motors_func[3]->Get_yconst();
            break;

        default:
            break;
    }

    return 4 * CH_C_PI;
}

double CuriosityRover::GetRoverMass() {
    double tot_mass = 0.0;
    tot_mass = tot_mass + m_chassis->GetBody()->GetMass();

    for (int i = 0; i < 6; i++) {
        tot_mass = tot_mass + m_wheels[i]->GetBody()->GetMass();
    }

    for (int i = 0; i < 4; i++) {
        tot_mass = tot_mass + m_arms[i]->GetBody()->GetMass();
    }

    for (int i = 0; i < 4; i++) {
        tot_mass = tot_mass + m_steers[i]->GetBody()->GetMass();
    }

    for (int i = 0; i < 3; i++) {
        tot_mass = tot_mass + m_balancers[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double CuriosityRover::GetWheelMass() {
    return m_wheels[0]->GetBody()->GetMass();
}

void CuriosityRover::Update() {
    UpdateDCMotorControl();
}

// Linear DC motor control -> uses a power shaft and a driven shaft for motor structure
void CuriosityRover::UpdateDCMotorControl() {
    if (m_dc_motor_control == false) {
        return;
    }

    std::vector<double> speed_reading;
    std::vector<double> target_torque;
    for (int i = 0; i < 6; i++) {
        if (i == 0 || i == 2 || i == 4) {
            speed_reading.push_back(-m_driven_shafts[i]->GetPos_dt());
        } else {
            speed_reading.push_back(m_driven_shafts[i]->GetPos_dt());
        }

        if (speed_reading[i] > m_no_load_speed[i]) {
            target_torque.push_back(0);
        } else if (speed_reading[i] < 0) {
            target_torque.push_back(m_stall_torque[i]);
        } else {
            double t = m_stall_torque[i] * ((m_no_load_speed[i] - speed_reading[i]) / m_no_load_speed[i]);
            target_torque.push_back(t);
        }
    }

    for (int i = 0; i < 6; i++) {
        if (i == 0 || i == 2 || i == 4) {
            m_power_shafts[i]->SetAppliedTorque(target_torque[i]);
        } else {
            m_power_shafts[i]->SetAppliedTorque(-target_torque[i]);
        }
    }
}

}  // namespace curiosity
}  // namespace chrono
