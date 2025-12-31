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

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

#include "chrono/physics/ChMassProperties.h"

#include "chrono_models/robot/turtlebot/Turtlebot.h"

namespace chrono {
namespace turtlebot {

// =============================================================================
// Create default contact material for the robot
std::shared_ptr<ChContactMaterial> DefaultContactMaterial(ChContactMethod contact_method) {
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
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
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
            return std::shared_ptr<ChContactMaterial>();
    }
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddRevoluteJoint(std::shared_ptr<ChBodyAuxRef> body_1,
                      std::shared_ptr<ChBodyAuxRef> body_2,
                      std::shared_ptr<ChBodyAuxRef> chassis,
                      ChSystem* system,
                      const ChVector3d& rel_joint_pos,
                      const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    revo->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(revo);
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddRevoluteJoint(std::shared_ptr<ChBodyEasyBox> body_1,
                      std::shared_ptr<ChBodyAuxRef> body_2,
                      std::shared_ptr<ChBodyAuxRef> chassis,
                      ChSystem* system,
                      const ChVector3d& rel_joint_pos,
                      const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    revo->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(revo);
}

// Add a revolute joint between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the revolute point
void AddLockJoint(std::shared_ptr<ChBodyAuxRef> body_1,
                  std::shared_ptr<ChBodyAuxRef> body_2,
                  std::shared_ptr<ChBodyAuxRef> chassis,
                  ChSystem* system,
                  const ChVector3d& rel_joint_pos,
                  const ChQuaternion<>& rel_joint_rot) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    // auto revo = chrono_types::make_shared<ChLinkLockRevolute>();
    auto revo = chrono_types::make_shared<ChLinkLockLock>();
    revo->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(revo);
}

// Add a rotational speed controlled motor between body_1 and body_2
// rel_joint_pos and rel_joint_rot are the position and the rotation of the motor
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotor(std::shared_ptr<ChBody> body_1,
                                                   std::shared_ptr<ChBodyAuxRef> body_2,
                                                   std::shared_ptr<ChBodyAuxRef> chassis,
                                                   ChSystem* system,
                                                   const ChVector3d& rel_joint_pos,
                                                   const ChQuaternion<>& rel_joint_rot,
                                                   std::shared_ptr<ChFunctionConst> speed_func) {
    const ChFrame<>& X_GP = chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);         // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                         // global -> child

    auto motor_angle = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_angle->Initialize(body_1, body_2, ChFrame<>(X_GC.GetCoordsys().pos, X_GC.GetCoordsys().rot));
    system->AddLink(motor_angle);
    motor_angle->SetSpeedFunction(speed_func);
    return motor_angle;
}

// ===============================================================================
Turtlebot_Part::Turtlebot_Part(const std::string& name,
                               bool fixed,
                               std::shared_ptr<ChContactMaterial> mat,
                               ChSystem* system,
                               const ChVector3d& body_pos,
                               const ChQuaternion<>& body_rot,
                               std::shared_ptr<ChBodyAuxRef> chassis_body,
                               bool collide) {
    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetName(name + "_body");
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
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
    trimesh->Transform(m_offset, ChMatrix33<>(1));
    auto trimesh_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_mesh_name);
    trimesh_shape->SetMutable(false);
    m_body->AddVisualShape(trimesh_shape);
    return;
}

void Turtlebot_Part::EnableCollision(bool state) {
    m_collide = state;
}

// Add collision assets
void Turtlebot_Part::AddCollisionShapes() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(m_offset, ChMatrix33<>(1));

    auto shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(m_mat, trimesh, false, false, 0.005);
    m_body->AddCollisionShape(shape);
    m_body->EnableCollision(m_collide);
}

// =============================================================================
// Robot Chassis
Turtlebot_Chassis::Turtlebot_Chassis(const std::string& name,
                                     bool fixed,
                                     std::shared_ptr<ChContactMaterial> mat,
                                     ChSystem* system,
                                     const ChVector3d& body_pos,
                                     const ChQuaternion<>& body_rot,
                                     bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, NULL, collide) {
    m_mesh_name = "chassis";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Turtlebot_Chassis::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);
    m_body->SetFrameRefToAbs(ChFrame<>(m_pos, m_rot));
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::ACTIVE_WHEEL);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_Chassis::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_Chassis::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_ActiveWheel::Turtlebot_ActiveWheel(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<ChContactMaterial> mat,
                                             ChSystem* system,
                                             const ChVector3d& body_pos,
                                             const ChQuaternion<>& body_rot,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "active_wheel";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void Turtlebot_ActiveWheel::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::ACTIVE_WHEEL);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_ActiveWheel::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_ActiveWheel::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_PassiveWheel::Turtlebot_PassiveWheel(const std::string& name,
                                               bool fixed,
                                               std::shared_ptr<ChContactMaterial> mat,
                                               ChSystem* system,
                                               const ChVector3d& body_pos,
                                               const ChQuaternion<>& body_rot,
                                               std::shared_ptr<ChBodyAuxRef> chassis,
                                               bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "passive_wheel";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
}

void Turtlebot_PassiveWheel::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::PASSIVE_WHEEL);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_PassiveWheel::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_PassiveWheel::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_Rod_Short::Turtlebot_Rod_Short(const std::string& name,
                                         bool fixed,
                                         std::shared_ptr<ChContactMaterial> mat,
                                         ChSystem* system,
                                         const ChVector3d& body_pos,
                                         const ChQuaternion<>& body_rot,
                                         std::shared_ptr<ChBodyAuxRef> chassis,
                                         bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "support_rod_short";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Turtlebot_Rod_Short::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::BOTTOM_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::MIDDLE_PLATE);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_Rod_Short::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_Rod_Short::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_BottomPlate::Turtlebot_BottomPlate(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<ChContactMaterial> mat,
                                             ChSystem* system,
                                             const ChVector3d& body_pos,
                                             const ChQuaternion<>& body_rot,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "plate_1";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 20;
}

void Turtlebot_BottomPlate::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::BOTTOM_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::ROD);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_BottomPlate::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_BottomPlate::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_MiddlePlate::Turtlebot_MiddlePlate(const std::string& name,
                                             bool fixed,
                                             std::shared_ptr<ChContactMaterial> mat,
                                             ChSystem* system,
                                             const ChVector3d& body_pos,
                                             const ChQuaternion<>& body_rot,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "plate_2";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 20;
}

void Turtlebot_MiddlePlate::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::MIDDLE_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::ROD);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_MiddlePlate::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_MiddlePlate::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_TopPlate::Turtlebot_TopPlate(const std::string& name,
                                       bool fixed,
                                       std::shared_ptr<ChContactMaterial> mat,
                                       ChSystem* system,
                                       const ChVector3d& body_pos,
                                       const ChQuaternion<>& body_rot,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "plate_3";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 20;
}

void Turtlebot_TopPlate::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::TOP_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::ROD);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::MIDDLE_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::BOTTOM_PLATE);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_TopPlate::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_TopPlate::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
Turtlebot_Rod_Long::Turtlebot_Rod_Long(const std::string& name,
                                       bool fixed,
                                       std::shared_ptr<ChContactMaterial> mat,
                                       ChSystem* system,
                                       const ChVector3d& body_pos,
                                       const ChQuaternion<>& body_rot,
                                       std::shared_ptr<ChBodyAuxRef> chassis,
                                       bool collide)
    : Turtlebot_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "support_rod_long";
    m_offset = ChVector3d(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 100;
}

void Turtlebot_Rod_Long::Initialize() {
    auto vis_mesh_file = GetChronoDataFile("robot/turtlebot/" + m_mesh_name + ".obj");
    auto trimesh = ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, false, false);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(1));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                    // if meshes are not watertight

    double mmass;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

    m_body->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    m_body->SetMass(mmass * m_density);
    m_body->SetInertiaXX(m_density * principal_I);

    // set relative position to chassis
    const ChFrame<>& X_GP = m_chassis->GetFrameRefToAbs();  // global -> parent
    ChFrame<> X_PC(m_pos, m_rot);                           // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                           // global -> child
    m_body->SetFrameRefToAbs(X_GC);
    m_body->SetFixed(m_fixed);

    AddCollisionShapes();

    m_body->GetCollisionModel()->SetFamily(CollisionFamily::ROD);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::CHASSIS);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::BOTTOM_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::MIDDLE_PLATE);
    m_body->GetCollisionModel()->DisallowCollisionsWith(CollisionFamily::ROD);

    AddVisualizationAssets();

    m_system->Add(m_body);
}

void Turtlebot_Rod_Long::EnableCollision(bool state) {
    m_collide = state;
    m_body->EnableCollision(state);
}

void Turtlebot_Rod_Long::Translate(const ChVector3d& shift) {
    m_body->SetPos(m_body->GetPos() + shift);
}

// ==========================================================
// Turtlebot Class for the complete robot model
TurtleBot::TurtleBot(ChSystem* system,
                     const ChVector3d& robot_pos,
                     const ChQuaternion<>& robot_rot,
                     std::shared_ptr<ChContactMaterial> wheel_mat)
    : m_system(system), m_robot_pos(robot_pos), m_robot_rot(robot_rot), m_wheel_material(wheel_mat) {
    // Set default collision model envelope commensurate with model dimensions.
    // Note that an SMC system automatically sets envelope to 0.
    auto contact_method = m_system->GetContactMethod();
    if (contact_method == ChContactMethod::NSC) {
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.01);
        ChCollisionModel::SetDefaultSuggestedMargin(0.005);
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
    m_drive_wheels.push_back(chrono_types::make_shared<Turtlebot_ActiveWheel>(
        "LDWheel", false, m_wheel_material, m_system, ChVector3d(dwx, +dwy, dwz), ChQuaternion<>(1, 0, 0, 0),
        m_chassis->GetBody(), true));
    m_drive_wheels.push_back(chrono_types::make_shared<Turtlebot_ActiveWheel>(
        "RDWheel", false, m_wheel_material, m_system, ChVector3d(dwx, -dwy, dwz), ChQuaternion<>(1, 0, 0, 0),
        m_chassis->GetBody(), true));

    // passive driven wheels' positions relative to the chassis
    double pwx = 0.11505;
    double pwy = 0;
    double pwz = 0.02015;

    m_passive_wheels.push_back(chrono_types::make_shared<Turtlebot_PassiveWheel>(
        "FPWheel", false, m_wheel_material, m_system, ChVector3d(pwx, pwy, pwz), ChQuaternion<>(1, 0, 0, 0),
        m_chassis->GetBody(), true));
    m_passive_wheels.push_back(chrono_types::make_shared<Turtlebot_PassiveWheel>(
        "RPWheel", false, m_wheel_material, m_system, ChVector3d(-pwx, pwy, pwz), ChQuaternion<>(1, 0, 0, 0),
        m_chassis->GetBody(), true));

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

    m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "0-bottom-rod", false, m_wheel_material, m_system, ChVector3d(rod_s_0_x, rod_s_0_y, rod_s_0_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "1-bottom-rod", false, m_wheel_material, m_system, ChVector3d(rod_s_1_x, rod_s_1_y, rod_s_1_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "2-bottom-rod", false, m_wheel_material, m_system, ChVector3d(rod_s_2_x, rod_s_2_y, rod_s_2_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "3-bottom-rod", false, m_wheel_material, m_system, ChVector3d(rod_s_3_x, rod_s_3_y, rod_s_3_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "4-bottom-rod", false, m_wheel_material, m_system, ChVector3d(rod_s_4_x, rod_s_4_y, rod_s_4_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_1st_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "5-bottom-rod", false, m_wheel_material, m_system, ChVector3d(rod_s_5_x, rod_s_5_y, rod_s_5_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));

    // add the bottom plate
    double bt_plate_x = 0;
    double bt_plate_y = 0;
    double bt_plate_z = 0.14615;

    m_bottom_plate = chrono_types::make_shared<Turtlebot_BottomPlate>(
        "bottom_plate", false, m_wheel_material, m_system, ChVector3d(bt_plate_x, bt_plate_y, bt_plate_z),
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

    m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "0-middle-rod", false, m_wheel_material, m_system, ChVector3d(rod_m_0_x, rod_m_0_y, rod_m_0_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "1-middle-rod", false, m_wheel_material, m_system, ChVector3d(rod_m_1_x, rod_m_1_y, rod_m_1_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "2-middle-rod", false, m_wheel_material, m_system, ChVector3d(rod_m_2_x, rod_m_2_y, rod_m_2_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "3-middle-rod", false, m_wheel_material, m_system, ChVector3d(rod_m_3_x, rod_m_3_y, rod_m_3_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "4-middle-rod", false, m_wheel_material, m_system, ChVector3d(rod_m_4_x, rod_m_4_y, rod_m_4_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_2nd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Short>(
        "5-middle-rod", false, m_wheel_material, m_system, ChVector3d(rod_m_5_x, rod_m_5_y, rod_m_5_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));

    // add the middle plate
    double mi_plate_x = 0;
    double mi_plate_y = 0;
    double mi_plate_z = 0.20015;
    m_middle_plate = chrono_types::make_shared<Turtlebot_MiddlePlate>(
        "middle_plate", false, m_wheel_material, m_system, ChVector3d(mi_plate_x, mi_plate_y, mi_plate_z),
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

    m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
        "0-top-rod", false, m_wheel_material, m_system, ChVector3d(rod_u_0_x, rod_u_0_y, rod_u_0_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
        "1-top-rod", false, m_wheel_material, m_system, ChVector3d(rod_u_1_x, rod_u_1_y, rod_u_1_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
        "2-top-rod", false, m_wheel_material, m_system, ChVector3d(rod_u_2_x, rod_u_2_y, rod_u_2_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
        "3-top-rod", false, m_wheel_material, m_system, ChVector3d(rod_u_3_x, rod_u_3_y, rod_u_3_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
        "4-top-rod", false, m_wheel_material, m_system, ChVector3d(rod_u_4_x, rod_u_4_y, rod_u_4_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));
    m_3rd_level_rods.push_back(chrono_types::make_shared<Turtlebot_Rod_Long>(
        "5-top-rod", false, m_wheel_material, m_system, ChVector3d(rod_u_5_x, rod_u_5_y, rod_u_5_z),
        ChQuaternion<>(1, 0, 0, 0), m_chassis->GetBody(), true));

    // add the top plate
    double top_plate_x = 0;
    double top_plate_y = 0;
    double top_plate_z = 0.40615;
    m_top_plate = chrono_types::make_shared<Turtlebot_TopPlate>("top_plate", false, m_wheel_material, m_system,
                                                                ChVector3d(top_plate_x, top_plate_y, top_plate_z),
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
    auto const_speed_function_l = chrono_types::make_shared<ChFunctionConst>(-CH_PI);
    auto const_speed_function_r = chrono_types::make_shared<ChFunctionConst>(-CH_PI);
    m_motors_func.push_back(const_speed_function_l);
    m_motors_func.push_back(const_speed_function_r);

    ChQuaternion<> z2y = QuatFromAngleX(CH_PI_2);
    ChQuaternion<> z2x = QuatFromAngleY(-CH_PI_2);

    m_motors.push_back(AddMotor(m_drive_wheels[0]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                                ChVector3d(dwx, dwy, dwz), z2y, const_speed_function_l));
    AddRevoluteJoint(m_passive_wheels[0]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                     ChVector3d(pwx, pwy, pwz), z2x);

    m_motors.push_back(AddMotor(m_drive_wheels[1]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                                ChVector3d(dwx, -dwy, dwz), z2y, const_speed_function_r));
    AddRevoluteJoint(m_passive_wheels[1]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                     ChVector3d(-pwx, pwy, pwz), z2x);

    // add fixity on all rods and plates
    // There are six constraints needed:
    // chassis -> bottom rods
    // bottom rods -> bottom plate
    // bottom plate -> middle rods
    // middle rods -> middle plate
    // middle plate -> top rods
    // top rods -> top plate

    ChVector3d bottom_rod_rel_pos[] = {ChVector3d(rod_s_0_x, rod_s_0_y, rod_s_0_z),  //
                                       ChVector3d(rod_s_1_x, rod_s_1_y, rod_s_1_z),  //
                                       ChVector3d(rod_s_2_x, rod_s_2_y, rod_s_2_z),  //
                                       ChVector3d(rod_s_3_x, rod_s_3_y, rod_s_3_z),  //
                                       ChVector3d(rod_s_4_x, rod_s_4_y, rod_s_4_z),  //
                                       ChVector3d(rod_s_5_x, rod_s_5_y, rod_s_5_z)};
    ChVector3d middle_rod_rel_pos[] = {ChVector3d(rod_m_0_x, rod_m_0_y, rod_m_0_z),  //
                                       ChVector3d(rod_m_1_x, rod_m_1_y, rod_m_1_z),  //
                                       ChVector3d(rod_m_2_x, rod_m_2_y, rod_m_2_z),  //
                                       ChVector3d(rod_m_3_x, rod_m_3_y, rod_m_3_z),  //
                                       ChVector3d(rod_m_4_x, rod_m_4_y, rod_m_4_z),  //
                                       ChVector3d(rod_m_5_x, rod_m_5_y, rod_m_5_z)};
    ChVector3d top_rod_rel_pos[] = {ChVector3d(rod_u_0_x, rod_u_0_y, rod_u_0_z),  //
                                    ChVector3d(rod_u_1_x, rod_u_1_y, rod_u_1_z),  //
                                    ChVector3d(rod_u_2_x, rod_u_2_y, rod_u_2_z),  //
                                    ChVector3d(rod_u_3_x, rod_u_3_y, rod_u_3_z),  //
                                    ChVector3d(rod_u_4_x, rod_u_4_y, rod_u_4_z),  //
                                    ChVector3d(rod_u_5_x, rod_u_5_y, rod_u_5_z)};

    for (int i = 0; i < 6; i++) {
        ChVector3d bottom_plate_rel_pos = bottom_rod_rel_pos[i] + ChVector3d(0, 0, 0.05);
        ChVector3d middle_plate_rel_pos = middle_rod_rel_pos[i] + ChVector3d(0, 0, 0.05);
        ChVector3d top_plate_rel_pos = top_rod_rel_pos[i] + ChVector3d(0, 0, 0.2);

        AddLockJoint(m_1st_level_rods[i]->GetBody(), m_chassis->GetBody(), m_chassis->GetBody(), m_system,
                     bottom_rod_rel_pos[i], ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_1st_level_rods[i]->GetBody(), m_bottom_plate->GetBody(), m_chassis->GetBody(), m_system,
                     bottom_plate_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_2nd_level_rods[i]->GetBody(), m_bottom_plate->GetBody(), m_chassis->GetBody(), m_system,
                     bottom_rod_rel_pos[i], ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_2nd_level_rods[i]->GetBody(), m_middle_plate->GetBody(), m_chassis->GetBody(), m_system,
                     middle_plate_rel_pos, ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_3rd_level_rods[i]->GetBody(), m_middle_plate->GetBody(), m_chassis->GetBody(), m_system,
                     top_rod_rel_pos[i], ChQuaternion<>(1, 0, 0, 0));
        AddLockJoint(m_3rd_level_rods[i]->GetBody(), m_top_plate->GetBody(), m_chassis->GetBody(), m_system,
                     top_plate_rel_pos, ChQuaternion<>(1, 0, 0, 0));
    }
}

void TurtleBot::SetMotorSpeed(double rad_speed, WheelID id) {
    m_motors_func[id]->SetConstant(rad_speed);
}

ChVector3d TurtleBot::GetActiveWheelSpeed(WheelID id) {
    return m_drive_wheels[id]->GetBody()->GetPosDt();
}

ChVector3d TurtleBot::GetActiveWheelAngVel(WheelID id) {
    return m_drive_wheels[id]->GetBody()->GetAngVelParent();
}

}  // namespace turtlebot
}  // namespace chrono
