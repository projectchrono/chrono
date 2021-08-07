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
// Turtlebot Robot Class
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

#include "chrono_models/robot/turtlebot/Turtlebot.h"

namespace chrono {
namespace turtlebot {

// =============================================================================
// Create default contact material for the robot
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
    revo->Initialize(body_1, body_2, ChCoordsys<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    system->AddLink(revo);
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddRevoluteJoint(std::shared_ptr<ChBodyEasyBox> body_1,
                      std::shared_ptr<ChBodyAuxRef> body_2,
                      std::shared_ptr<ChBodyAuxRef> chassis,
                      ChSystem* system,
                      const ChVector<>& rel_joint_pos,
                      const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);            // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child

    auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
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

// Add a rotational speed controlled motor between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the motor
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotor(std::shared_ptr<ChBody> body_1,
                                                   std::shared_ptr<ChBodyAuxRef> body_2,
                                                   std::shared_ptr<ChBodyAuxRef> chassis,
                                                   ChSystem* system,
                                                   const ChVector<>& rel_joint_pos,
                                                   const ChQuaternion<>& rel_joint_rot,
                                                   std::shared_ptr<ChFunction_Const> speed_func) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);            // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child

    auto motor_angle = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_angle->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    system->AddLink(motor_angle);
    motor_angle->SetSpeedFunction(speed_func);
    return motor_angle;
}

// ===============================================================================
Turtlebot_Part::Turtlebot_Part(const std::string& name,
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

// Create Visulization assets
void Turtlebot_Part::AddVisualizationAssets() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

void Turtlebot_Part::SetCollide(bool state) {
    m_collide = state;
}

// Add collision assets
void Turtlebot_Part::AddCollisionShapes() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
    auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
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
// Robot Chassis
Turtlebot_Chassis::Turtlebot_Chassis(const std::string& name,
                                     bool fixed,
                                     std::shared_ptr<ChMaterialSurface> mat,
                                     ChSystem* system,
                                     const ChVector<>& body_pos,
                                     const ChQuaternion<>& body_rot,
                                     bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, NULL, collide) {
    m_mesh_name = "chassis";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Turtlebot_Chassis::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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
    m_body->SetFrame_REF_to_abs(ChFrame<>(m_pos, m_rot));
    m_body->SetBodyFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::ACTIVE_WHEEL);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_Chassis::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_Chassis::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_ActiveWheel::Turtlebot_ActiveWheel(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<ChMaterialSurface> mat,
                                             ChSystem* system,
                                             const ChVector<>& body_pos,
                                             const ChQuaternion<>& body_rot,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "active_wheel";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void Turtlebot_ActiveWheel::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::ACTIVE_WHEEL);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_ActiveWheel::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_ActiveWheel::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_PassiveWheel::Turtlebot_PassiveWheel(const std::string& name,
                                               bool fixed,
                                               std::shared_ptr<ChMaterialSurface> mat,
                                               ChSystem* system,
                                               const ChVector<>& body_pos,
                                               const ChQuaternion<>& body_rot,
                                               std::shared_ptr<ChBodyAuxRef> chassis,
                                               bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "passive_wheel";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void Turtlebot_PassiveWheel::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::PASSIVE_WHEEL);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_PassiveWheel::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_PassiveWheel::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_Rod_Short::Turtlebot_Rod_Short(const std::string& name,
                                         bool fixed,
                                         std::shared_ptr<ChMaterialSurface> mat,
                                         ChSystem* system,
                                         const ChVector<>& body_pos,
                                         const ChQuaternion<>& body_rot,
                                         std::shared_ptr<ChBodyAuxRef> chassis,
                                         bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "support_rod_short";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Turtlebot_Rod_Short::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::BOTTOM_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::MIDDLE_PLATE);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_Rod_Short::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_Rod_Short::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_BottomPlate::Turtlebot_BottomPlate(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<ChMaterialSurface> mat,
                                             ChSystem* system,
                                             const ChVector<>& body_pos,
                                             const ChQuaternion<>& body_rot,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "plate_1";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 20;
}

void Turtlebot_BottomPlate::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::BOTTOM_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_BottomPlate::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_BottomPlate::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_MiddlePlate::Turtlebot_MiddlePlate(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<ChMaterialSurface> mat,
                                             ChSystem* system,
                                             const ChVector<>& body_pos,
                                             const ChQuaternion<>& body_rot,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "plate_2";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 20;
}

void Turtlebot_MiddlePlate::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::MIDDLE_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_MiddlePlate::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_MiddlePlate::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_TopPlate::Turtlebot_TopPlate(const std::string& name,
                                       bool fixed,
                                       std::shared_ptr<ChMaterialSurface> mat,
                                       ChSystem* system,
                                       const ChVector<>& body_pos,
                                       const ChQuaternion<>& body_rot,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "plate_3";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 20;
}

void Turtlebot_TopPlate::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::TOP_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::MIDDLE_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::BOTTOM_PLATE);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_TopPlate::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_TopPlate::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_Rod_Long::Turtlebot_Rod_Long(const std::string& name,
                                       bool fixed,
                                       std::shared_ptr<ChMaterialSurface> mat,
                                       ChSystem* system,
                                       const ChVector<>& body_pos,
                                       const ChQuaternion<>& body_rot,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "support_rod_long";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Turtlebot_Rod_Long::Initialize() {
    std::string vis_mesh_file = "robot/turtlebot/" + m_mesh_name + ".obj";
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

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::BOTTOM_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::MIDDLE_PLATE);
    m_body->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(CollisionFamily::ROD);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_Rod_Long::SetCollide(bool state) {
    m_collide = state;
    m_body->SetCollide(state);
}

void Turtlebot_Rod_Long::Translate(const ChVector<>& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Turtlebot Class for the complete robot model
TurtleBot::TurtleBot(ChSystem* system,
                     const ChVector<>& robot_pos,
                     const ChQuaternion<>& robot_rot,
                     std::shared_ptr<ChMaterialSurface> wheel_mat)
    : m_system(system), m_robot_pos(robot_pos), m_robot_rot(robot_rot), m_wheel_material(wheel_mat) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == ChContactMethod::NSC) {
        collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        collision::ChCollisionModel::SetDefaultSuggestedMargin(0.005);
    }

    // Create the contact materials
    m_chassis_material = DefaultContactMaterial(contact_method);
    if (!m_wheel_material)
        m_wheel_material = DefaultContactMaterial(contact_method);

    Create();
}

TurtleBot::~TurtleBot() {}

void TurtleBot::Create() {
    // initialize robot chassis
    m_chassis = chrono_types::make_shared<Turtlebot_Chassis>("chassis", false, m_chassis_material, m_system,
                                                             m_robot_pos, m_robot_rot, true);

    // active drive wheels' positions relative to the chassis
    double dwx = 0;
    double dwy = 0.11505;
    double dwz = 0.03735;
    for (int i = 0; i < 2; i++) {
        if (i == 0) {
            m_drive_wheels.push_back(chrono_types::make_shared<Turtlebot_ActiveWheel>(
                "LDWheel", false, m_wheel_material, m_system, ChVector<>(dwx, +dwy, dwz), ChQuaternion<>(1, 0, 0, 0),
                m_chassis->GetBody(), true));
        }

        if (i == 1) {
            m_drive_wheels.push_back(chrono_types::make_shared<Turtlebot_ActiveWheel>(
                "RDWheel", false, m_wheel_material, m_system, ChVector<>(dwx, -dwy, dwz), ChQuaternion<>(1, 0, 0, 0),
                m_chassis->GetBody(), true));
        }
    }

    // passive driven wheels' positions relative to the chassis
    double pwx = 0.11505;
    double pwy = 0;
    double pwz = 0.02015;

    for (int i = 0; i < 2; i++) {
        if (i == 0) {
            m_passive_wheels.push_back(chrono_types::make_shared<Turtlebot_PassiveWheel>(
                "FPWheel", false, m_wheel_material, m_system, ChVector<>(pwx, pwy, pwz), ChQuaternion<>(1, 0, 0, 0),
                m_chassis->GetBody(), true));
        }

        if (i == 1) {
            m_passive_wheels.push_back(chrono_types::make_shared<Turtlebot_PassiveWheel>(
                "RPWheel", false, m_wheel_material, m_system, ChVector<>(-pwx, pwy, pwz), ChQuaternion<>(1, 0, 0, 0),
                m_chassis->GetBody(), true));
        }
    }

    // create the first level supporting rod
    double rod_s_0_x = -0.0565;
    double rod_s_0_y = 0.11992;
    double rod_s_0_z = 0.09615;

    double rod_s_1_x = 0.0535;
    double rod_s_1_y = 0.11992;
    double rod_s_1_z = 0.09615;

    double rod_s_2_x = 0.11850;
    double rod_s_2_y = 0.08192;
    double rod_s_2_z = 0.09615;

    double rod_s_3_x = 0.11850;
    double rod_s_3_y = -0.08192;
    double rod_s_3_z = 0.09615;

    double rod_s_4_x = 0.0535;
    double rod_s_4_y = -0.11992;
    double rod_s_4_z = 0.09615;

    double rod_s_5_x = -0.0565;
    double rod_s_5_y = -0.11992;
    double rod_s_5_z = 0.09615;

    for (int i = 0; i < 6; i++) {
        if (i == 0) {
            m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "0-bottom-rod", false, m_wheel_material, m_system, ChVector<>(rod_s_0_x, rod_s_0_y, rod_s_0_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 1) {
            m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "1-bottom-rod", false, m_wheel_material, m_system, ChVector<>(rod_s_1_x, rod_s_1_y, rod_s_1_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 2) {
            m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "2-bottom-rod", false, m_wheel_material, m_system, ChVector<>(rod_s_2_x, rod_s_2_y, rod_s_2_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 3) {
            m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "3-bottom-rod", false, m_wheel_material, m_system, ChVector<>(rod_s_3_x, rod_s_3_y, rod_s_3_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 4) {
            m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "4-bottom-rod", false, m_wheel_material, m_system, ChVector<>(rod_s_4_x, rod_s_4_y, rod_s_4_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 5) {
            m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "5-bottom-rod", false, m_wheel_material, m_system, ChVector<>(rod_s_5_x, rod_s_5_y, rod_s_5_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }
    }

    // add the bottom plate
    double bt_plate_x = 0;
    double bt_plate_y = 0;
    double bt_plate_z = 0.14615;

    m_bottom_plate = chrono_types::make_shared<Turtlebot_BottomPlate>(
        "bottom_plate", false, m_wheel_material, m_system, ChVector<>(bt_plate_x, bt_plate_y, bt_plate_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true);

    // create the second level support rod
    double rod_m_0_x = -0.10394;
    double rod_m_0_y = 0.09792;
    double rod_m_0_z = 0.15015;

    double rod_m_1_x = -0.0015;
    double rod_m_1_y = 0.16192;
    double rod_m_1_z = 0.15015;

    double rod_m_2_x = 0.0687;
    double rod_m_2_y = 0.13132;
    double rod_m_2_z = 0.15015;

    double rod_m_3_x = 0.0687;
    double rod_m_3_y = -0.13132;
    double rod_m_3_z = 0.15015;

    double rod_m_4_x = -0.0015;
    double rod_m_4_y = -0.16192;
    double rod_m_4_z = 0.15015;

    double rod_m_5_x = -0.10394;
    double rod_m_5_y = -0.09792;
    double rod_m_5_z = 0.15015;

    for (int i = 0; i < 6; i++) {
        if (i == 0) {
            m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "0-middle-rod", false, m_wheel_material, m_system, ChVector<>(rod_m_0_x, rod_m_0_y, rod_m_0_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 1) {
            m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "1-middle-rod", false, m_wheel_material, m_system, ChVector<>(rod_m_1_x, rod_m_1_y, rod_m_1_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 2) {
            m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "2-middle-rod", false, m_wheel_material, m_system, ChVector<>(rod_m_2_x, rod_m_2_y, rod_m_2_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 3) {
            m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "3-middle-rod", false, m_wheel_material, m_system, ChVector<>(rod_m_3_x, rod_m_3_y, rod_m_3_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 4) {
            m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "4-middle-rod", false, m_wheel_material, m_system, ChVector<>(rod_m_4_x, rod_m_4_y, rod_m_4_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 5) {
            m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
                "5-middle-rod", false, m_wheel_material, m_system, ChVector<>(rod_m_5_x, rod_m_5_y, rod_m_5_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }
    }

    // add the middle plate
    double mi_plate_x = 0;
    double mi_plate_y = 0;
    double mi_plate_z = 0.20015;
    m_middle_plate = chrono_types::make_shared<Turtlebot_MiddlePlate>(
        "middle_plate", false, m_wheel_material, m_system, ChVector<>(mi_plate_x, mi_plate_y, mi_plate_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true);

    // create the third level support rod
    double rod_u_0_x = -0.10394;
    double rod_u_0_y = 0.09792;
    double rod_u_0_z = 0.20615;

    double rod_u_1_x = -0.0015;
    double rod_u_1_y = 0.16192;
    double rod_u_1_z = 0.20615;

    double rod_u_2_x = 0.0687;
    double rod_u_2_y = 0.13132;
    double rod_u_2_z = 0.20615;

    double rod_u_3_x = 0.0687;
    double rod_u_3_y = -0.13132;
    double rod_u_3_z = 0.20615;

    double rod_u_4_x = -0.0015;
    double rod_u_4_y = -0.16192;
    double rod_u_4_z = 0.20615;

    double rod_u_5_x = -0.10394;
    double rod_u_5_y = -0.09792;
    double rod_u_5_z = 0.20615;

    for (int i = 0; i < 6; i++) {
        if (i == 0) {
            m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
                "0-top-rod", false, m_wheel_material, m_system, ChVector<>(rod_u_0_x, rod_u_0_y, rod_u_0_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 1) {
            m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
                "1-top-rod", false, m_wheel_material, m_system, ChVector<>(rod_u_1_x, rod_u_1_y, rod_u_1_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 2) {
            m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
                "2-top-rod", false, m_wheel_material, m_system, ChVector<>(rod_u_2_x, rod_u_2_y, rod_u_2_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 3) {
            m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
                "3-top-rod", false, m_wheel_material, m_system, ChVector<>(rod_u_3_x, rod_u_3_y, rod_u_3_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 4) {
            m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
                "4-top-rod", false, m_wheel_material, m_system, ChVector<>(rod_u_4_x, rod_u_4_y, rod_u_4_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }

        if (i == 5) {
            m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
                "5-top-rod", false, m_wheel_material, m_system, ChVector<>(rod_u_5_x, rod_u_5_y, rod_u_5_z),
                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
        }
    }

    // add the top plate
    double top_plate_x = 0;
    double top_plate_y = 0;
    double top_plate_z = 0.40615;
    m_top_plate = chrono_types::make_shared<Turtlebot_TopPlate>("top_plate", false, m_wheel_material, m_system,
                                                                ChVector<>(top_plate_x, top_plate_y, top_plate_z),
                                                                ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true);
}

/// Initialize the complete rover and add all constraints
void TurtleBot::Initialize() {
    m_chassis->Initialize();
    m_bottom_plate->Initialize();
    m_middle_plate->Initialize();
    m_top_plate->Initialize();
    for (int i = 0; i < 2; i++) {
        m_drive_wheels[i]->Initialize();
        m_passive_wheels[i]->Initialize();
    }

    for (int i = 0; i < 6; i++) {
        m_1st_level_rods[i]->Initialize();
        m_2nd_level_rods[i]->Initialize();
        m_3rd_level_rods[i]->Initialize();
    }

    // redeclare necessary location variables
    double dwx = 0;
    double dwy = 0.11505;
    double dwz = 0.03735;

    double pwx = 0.11505;
    double pwy = 0;
    double pwz = 0.02015;

    double rod_s_0_x = -0.0565;
    double rod_s_0_y = 0.11992;
    double rod_s_0_z = 0.09615;

    double rod_s_1_x = 0.0535;
    double rod_s_1_y = 0.11992;
    double rod_s_1_z = 0.09615;

    double rod_s_2_x = 0.11850;
    double rod_s_2_y = 0.08192;
    double rod_s_2_z = 0.09615;

    double rod_s_3_x = 0.11850;
    double rod_s_3_y = -0.08192;
    double rod_s_3_z = 0.09615;

    double rod_s_4_x = 0.0535;
    double rod_s_4_y = -0.11992;
    double rod_s_4_z = 0.09615;

    double rod_s_5_x = -0.0565;
    double rod_s_5_y = -0.11992;
    double rod_s_5_z = 0.09615;

    double rod_m_0_x = -0.10394;
    double rod_m_0_y = 0.09792;
    double rod_m_0_z = 0.15015;

    double rod_m_1_x = -0.0015;
    double rod_m_1_y = 0.16192;
    double rod_m_1_z = 0.15015;

    double rod_m_2_x = 0.0687;
    double rod_m_2_y = 0.13132;
    double rod_m_2_z = 0.15015;

    double rod_m_3_x = 0.0687;
    double rod_m_3_y = -0.13132;
    double rod_m_3_z = 0.15015;

    double rod_m_4_x = -0.0015;
    double rod_m_4_y = -0.16192;
    double rod_m_4_z = 0.15015;

    double rod_m_5_x = -0.10394;
    double rod_m_5_y = -0.09792;
    double rod_m_5_z = 0.15015;

    double rod_u_0_x = -0.10394;
    double rod_u_0_y = 0.09792;
    double rod_u_0_z = 0.20615;

    double rod_u_1_x = -0.0015;
    double rod_u_1_y = 0.16192;
    double rod_u_1_z = 0.20615;

    double rod_u_2_x = 0.0687;
    double rod_u_2_y = 0.13132;
    double rod_u_2_z = 0.20615;

    double rod_u_3_x = 0.0687;
    double rod_u_3_y = -0.13132;
    double rod_u_3_z = 0.20615;

    double rod_u_4_x = -0.0015;
    double rod_u_4_y = -0.16192;
    double rod_u_4_z = 0.20615;

    double rod_u_5_x = -0.10394;
    double rod_u_5_y = -0.09792;
    double rod_u_5_z = 0.20615;

    // add motors and revolute joints on the active and passive wheels
    auto const_speed_function_l = chrono_types::make_shared<ChFunction_Const>(-CH_C_PI);
    auto const_speed_function_r = chrono_types::make_shared<ChFunction_Const>(-CH_C_PI);
    m_motors_func.push_back(const_speed_function_l);
    m_motors_func.push_back(const_speed_function_r);

    ChQuaternion<> z2y;
    z2y.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(1, 0, 0));

    ChQuaternion<> z2x;
    z2x.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(0, 1, 0));

    for (int i = 0; i < 2; i++) {
        ChVector<> active_rel_pos;
        ChVector<> passive_rel_pos;
        if (i == 0) {
            active_rel_pos = ChVector<>(dwx, dwy, dwz);
            passive_rel_pos = ChVector<>(pwx, pwy, pwz);
        } else {
            active_rel_pos = ChVector<>(dwx, -dwy, dwz);
            passive_rel_pos = ChVector<>(-pwx, pwy, pwz);
        }
        if (i == 0) {
            m_motors.push_back(AddMotor(m_drive_wheels[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(),
                                        m_system, active_rel_pos, z2y, const_speed_function_l));
        } else {
            m_motors.push_back(AddMotor(m_drive_wheels[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(),
                                        m_system, active_rel_pos, z2y, const_speed_function_r));
        }

        AddRevoluteJoint(m_passive_wheels[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                         passive_rel_pos, z2x);
    }

    // add fixity on all rods and plates
    // There are six constraints needed:
    // chassis -> bottom rods
    // bottom rods -> bottom plate
    // bottom plate -> middle rods
    // middle rods -> middle plate
    // middle plate -> top rods
    // top rods -> top plate
    for (int i = 0; i < 6; i++) {
        ChVector<> bottom_rod_rel_pos;
        ChVector<> bottom_plate_rel_pos;
        ChVector<> middle_rod_rel_pos;
        ChVector<> middle_plate_rel_pos;
        ChVector<> top_rod_rel_pos;
        ChVector<> top_plate_rel_pos;
        if (i == 0) {
            bottom_rod_rel_pos = ChVector<>(rod_s_0_x, rod_s_0_y, rod_s_0_z);
            middle_rod_rel_pos = ChVector<>(rod_m_0_x, rod_m_0_y, rod_m_0_z);
            top_rod_rel_pos = ChVector<>(rod_u_0_x, rod_u_0_y, rod_u_0_z);
            bottom_plate_rel_pos = bottom_rod_rel_pos + ChVector<>(0, 0, 0.05);
            middle_plate_rel_pos = middle_rod_rel_pos + ChVector<>(0, 0, 0.05);
            top_plate_rel_pos = top_rod_rel_pos + ChVector<>(0, 0, 0.2);
        } else if (i == 1) {
            bottom_rod_rel_pos = ChVector<>(rod_s_1_x, rod_s_1_y, rod_s_1_z);
            middle_rod_rel_pos = ChVector<>(rod_m_1_x, rod_m_1_y, rod_m_1_z);
            top_rod_rel_pos = ChVector<>(rod_u_1_x, rod_u_1_y, rod_u_1_z);
            bottom_plate_rel_pos = bottom_rod_rel_pos + ChVector<>(0, 0, 0.05);
            middle_plate_rel_pos = middle_rod_rel_pos + ChVector<>(0, 0, 0.05);
            top_plate_rel_pos = top_rod_rel_pos + ChVector<>(0, 0, 0.2);
        } else if (i == 2) {
            bottom_rod_rel_pos = ChVector<>(rod_s_2_x, rod_s_2_y, rod_s_2_z);
            middle_rod_rel_pos = ChVector<>(rod_m_2_x, rod_m_2_y, rod_m_2_z);
            top_rod_rel_pos = ChVector<>(rod_u_2_x, rod_u_2_y, rod_u_2_z);
            bottom_plate_rel_pos = bottom_rod_rel_pos + ChVector<>(0, 0, 0.05);
            middle_plate_rel_pos = middle_rod_rel_pos + ChVector<>(0, 0, 0.05);
            top_plate_rel_pos = top_rod_rel_pos + ChVector<>(0, 0, 0.2);
        } else if (i == 3) {
            bottom_rod_rel_pos = ChVector<>(rod_s_3_x, rod_s_3_y, rod_s_3_z);
            middle_rod_rel_pos = ChVector<>(rod_m_3_x, rod_m_3_y, rod_m_3_z);
            top_rod_rel_pos = ChVector<>(rod_u_3_x, rod_u_3_y, rod_u_3_z);
            bottom_plate_rel_pos = bottom_rod_rel_pos + ChVector<>(0, 0, 0.05);
            middle_plate_rel_pos = middle_rod_rel_pos + ChVector<>(0, 0, 0.05);
            top_plate_rel_pos = top_rod_rel_pos + ChVector<>(0, 0, 0.2);
        } else if (i == 4) {
            bottom_rod_rel_pos = ChVector<>(rod_s_4_x, rod_s_4_y, rod_s_4_z);
            middle_rod_rel_pos = ChVector<>(rod_m_4_x, rod_m_4_y, rod_m_4_z);
            top_rod_rel_pos = ChVector<>(rod_u_4_x, rod_u_4_y, rod_u_4_z);
            bottom_plate_rel_pos = bottom_rod_rel_pos + ChVector<>(0, 0, 0.05);
            middle_plate_rel_pos = middle_rod_rel_pos + ChVector<>(0, 0, 0.05);
            top_plate_rel_pos = top_rod_rel_pos + ChVector<>(0, 0, 0.2);
        } else if (i == 5) {
            bottom_rod_rel_pos = ChVector<>(rod_s_5_x, rod_s_5_y, rod_s_5_z);
            middle_rod_rel_pos = ChVector<>(rod_m_5_x, rod_m_5_y, rod_m_5_z);
            top_rod_rel_pos = ChVector<>(rod_u_5_x, rod_u_5_y, rod_u_5_z);
            bottom_plate_rel_pos = bottom_rod_rel_pos + ChVector<>(0, 0, 0.05);
            middle_plate_rel_pos = middle_rod_rel_pos + ChVector<>(0, 0, 0.05);
            top_plate_rel_pos = top_rod_rel_pos + ChVector<>(0, 0, 0.2);
        }

        AddLockJoint(m_1st_level_rods[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                     bottom_rod_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_1st_level_rods[i]->GetBody(), m_bottom_plate->GetBody(), m_chassis->GetBody(), m_system,
                     bottom_plate_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_2nd_level_rods[i]->GetBody(), m_bottom_plate->GetBody(), m_chassis->GetBody(), m_system,
                     bottom_rod_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_2nd_level_rods[i]->GetBody(), m_middle_plate->GetBody(), m_chassis->GetBody(), m_system,
                     middle_plate_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_3rd_level_rods[i]->GetBody(), m_middle_plate->GetBody(), m_chassis->GetBody(), m_system,
                     top_rod_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_3rd_level_rods[i]->GetBody(), m_top_plate->GetBody(), m_chassis->GetBody(), m_system,
                     top_plate_rel_pos, ChQuaternion<>(1, 0, 0, 0));
    }
}

void TurtleBot::SetMotorSpeed(double rad_speed, WheelID id) {
    m_motors_func[id]->Set_yconst(rad_speed);
}

ChVector<> TurtleBot::GetActiveWheelSpeed(WheelID id) {
    return m_drive_wheels[id]->GetBody()->GetPos_dt();
}

ChVector<> TurtleBot::GetActiveWheelAngVel(WheelID id) {
    return m_drive_wheels[id]->GetBody()->GetWvel_par();
}

}  // namespace turtlebot
}  // namespace chrono
