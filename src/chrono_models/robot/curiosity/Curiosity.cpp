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
// This class contains model for NASA's Curiosity Mars rover for NASA's 2024 Moon
// exploration mission.
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
    revo->Initialize(body_1, body_2, ChCoordsys<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
    system->AddLink(revo);
}

// Add a rotational speed controlled motor between body 'steer' and body 'wheel'
// rel_joint_pos and rel_joint_rot are the position and the rotation of the motor
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotor(std::shared_ptr<ChBody> steer,
                                                   std::shared_ptr<ChBodyAuxRef> wheel,
                                                   std::shared_ptr<ChBodyAuxRef> chassis,
                                                   ChSystem* system,
                                                   const ChVector<>& rel_joint_pos,
                                                   const ChQuaternion<>& rel_joint_rot,
                                                   std::shared_ptr<ChFunction_Const> speed_func) {
    const ChFrame<>& X_GP = chassis->GetFrame_REF_to_abs();  // global -> parent
    ChFrame<> X_PC(rel_joint_pos, rel_joint_rot);            // parent -> child
    ChFrame<> X_GC = X_GP * X_PC;                            // global -> child

    auto motor_angle = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    motor_angle->Initialize(steer, wheel, ChFrame<>(X_GC.GetCoord().pos, X_GC.GetCoord().rot));
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
                             bool collide)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, NULL, collide) {
    m_mesh_name = "curiosity_chassis";
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
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
                         bool collide)
    : Curiosity_Part(name, fixed, mat, system, body_pos, body_rot, chassis, collide) {
    m_mesh_name = "curiosity_wheel";
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
    if(side == 0){
        m_mesh_name = "curiosity_F_L_arm";
    }
    if(side == 1){
        m_mesh_name = "curiosity_F_R_arm";
    }
    if(side == 2){
        m_mesh_name = "curiosity_B_L_arm";
    }
    if(side == 3){
        m_mesh_name = "curiosity_B_R_arm";
    }
    
    m_offset = ChVector<>(0, 0, 0);
    m_color = ChColor(0.4f, 0.4f, 0.7f);
    m_density = 200;
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
    m_density = 200;
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
    if(side == 0){m_mesh_name = "curiosity_bar_L";}
    if(side == 1){m_mesh_name = "curiosity_bar_R";}
    if(side == 2){m_mesh_name = "curiosity_balancer";}
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
                    std::shared_ptr<ChMaterialSurface> wheel_mat)
    : m_system(system), m_rover_pos(rover_pos), m_rover_rot(rover_rot), m_wheel_material(wheel_mat), m_custom_wheel_mat(true){
    Create();
}

CuriosityRover::CuriosityRover(ChSystem* system, 
                    const ChVector<>& rover_pos, 
                    const ChQuaternion<>& rover_rot)
    : m_system(system), m_rover_pos(rover_pos), m_rover_rot(rover_rot), m_custom_wheel_mat(false){
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

    if(m_custom_wheel_mat == true){
        m_chassis_material = DefaultContactMaterial(contact_method);
        m_arm_material = DefaultContactMaterial(contact_method);
        m_steer_material = DefaultContactMaterial(contact_method);
    }else{
        // Create the contact materials (all with default properties)
        m_chassis_material = DefaultContactMaterial(contact_method);
        m_wheel_material = DefaultContactMaterial(contact_method);
        m_arm_material = DefaultContactMaterial(contact_method);
        m_steer_material = DefaultContactMaterial(contact_method);
    }

    // initialize rover chassis
    ChQuaternion<> body_rot;
    // TEMP TODO: move this body
    m_chassis = chrono_types::make_shared<Curiosity_Chassis>("chassis", true, m_chassis_material, m_system, m_rover_pos,
                                                         m_rover_rot, false);
    // initilize rover wheels
    ChVector<> wheel_rel_pos_lf = ChVector<>( 1.095, 1.063 ,0.249);
    ChVector<> wheel_rel_pos_rf = ChVector<>( 1.095, -1.063, 0.249);
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
            if (i == 4){
                wheel_pos = wheel_rel_pos_lb;
            }
            if (i == 5){
                wheel_pos = wheel_rel_pos_rb;
            }
            std::shared_ptr<Curiosity_Wheel> m_wheel;
            if (i == 1 || i == 3 || i == 5) {
                m_wheel = chrono_types::make_shared<Curiosity_Wheel>("wheel", true, m_wheel_material, m_system, wheel_pos,
                                                                 wheel_rot_ori_pi, m_chassis->GetBody(), true);
            } else {
                m_wheel = chrono_types::make_shared<Curiosity_Wheel>("wheel", true, m_wheel_material, m_system, wheel_pos,
                                                                 wheel_rot_ori, m_chassis->GetBody(), true);
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
            
            m_arm = chrono_types::make_shared<Curiosity_Arm>("rod", true, m_arm_material, m_system, rod_pos,
                                                                 rod_ori, m_chassis->GetBody(), false, i);

            m_arms.push_back(m_arm);
        }
    }

    ChVector<> sr_rel_pos_lf = ChVector<>( 1.095,1.063, 0.64);
    ChVector<> sr_rel_pos_rf = ChVector<>( 1.095,-1.063, 0.64);
    ChVector<> sr_rel_pos_lb = ChVector<>( -1.163,1.063,0.64);
    ChVector<> sr_rel_pos_rb = ChVector<>( -1.163,-1.063,0.64);
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
            
            if(i == 1 || i == 3){
                m_steer = chrono_types::make_shared<Curiosity_Steer>("steer", true, m_steer_material, m_system, steer_pos,
                                                                    steer_ori_pi, m_chassis->GetBody(), false);
            }else{
                m_steer = chrono_types::make_shared<Curiosity_Steer>("steer", true, m_steer_material, m_system, steer_pos,
                                                                    steer_ori, m_chassis->GetBody(), false);                
            }


            m_steers.push_back(m_steer);
        }
    }

    ChVector<> tr_rel_pos_l = ChVector<>( 0.214, 0.672, 1.144);
    ChVector<> tr_rel_pos_r = ChVector<>( 0.214, -0.672, 1.144);
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
            

            m_balancer = chrono_types::make_shared<Curiosity_Balancer>("balancer", true, m_steer_material, m_system, bal_pos,
                                                                    bal_ori, m_chassis->GetBody(), false, i);                




            m_balancers.push_back(m_balancer);
        }
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
    for (int i = 0; i < 3; i++){
        m_balancers[i]->Initialize();
    }
}

void CuriosityRover::SetMotorSpeed(double rad_speed, WheelID id) {
    //m_motors_func[id]->Set_yconst(rad_speed);
}


ChVector<> CuriosityRover::GetWheelSpeed(WheelID id) {
    return m_wheels[id]->GetBody()->GetPos_dt();
}

ChQuaternion<> CuriosityRover::GetWheelAngVel(WheelID id) {
    return m_wheels[id]->GetBody()->GetRot_dt();
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

std::shared_ptr<ChBodyAuxRef> CuriosityRover::GetWheelBody(WheelID id) {
    return m_wheels[id]->GetBody();
}

std::shared_ptr<ChBodyAuxRef> CuriosityRover::GetChassisBody() {
    return m_chassis->GetBody();
}

double CuriosityRover::GetRoverMass() {
    double tot_mass = 0.0;
    return tot_mass;
}

double CuriosityRover::GetWheelMass() {
    //return m_wheels[0]->GetBody()->GetMass();
    return 0.0;
}


}  // namespace curiosity
}  // namespace chrono
