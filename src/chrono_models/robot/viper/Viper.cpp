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
// Authors: Jason Zhou, Radu Serban
// =============================================================================
//
// NASA VIPER Lunar Rover Model Class.
// This class contains model for NASA's VIPER lunar rover for NASA's 2024 Moon
// exploration mission.
//
// =============================================================================
//
// RADU TODO:
// - Recheck kinematics of mechanism (for negative lift angle)
// - Forces and torques are reported relative to the part's centroidal frame.
//   Likely confusing for a user since all bodies are ChBodyAuxRef!
// - Consider using a torque motor instead of driveshafts
//   (for a driver that uses torque control)
//
// =============================================================================

#include <cmath>

#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono/motion_functions/ChFunction_Setpoint.h"

#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChShaftsBody.h"

#include "chrono/physics/ChInertiaUtils.h"

#include "chrono_models/robot/viper/Viper.h"

namespace chrono {
namespace viper {

// =============================================================================

const double Viper::m_max_steer_angle = CH_C_PI / 6;

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

// Add a revolute joint between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
void AddRevoluteJoint(std::shared_ptr<ChBody> body1,
                      std::shared_ptr<ChBody> body2,
                      std::shared_ptr<ViperChassis> chassis,
                      const ChVector<>& rel_pos,
                      const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create joint (DOF about Z axis of X_GC frame)
    auto joint = chrono_types::make_shared<ChLinkLockRevolute>();
    joint->Initialize(body1, body2, ChCoordsys<>(X_GC.GetPos(), X_GC.GetRot()));
    chassis->GetBody()->GetSystem()->AddLink(joint);
}

// Add a universal joint between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
void AddUniversalJoint(std::shared_ptr<ChBody> body1,
                       std::shared_ptr<ChBody> body2,
                       std::shared_ptr<ViperChassis> chassis,
                       const ChVector<>& rel_pos,
                       const ChQuaternion<>& rel_rot) {
    // Express relative frame in global
    ChFrame<> X_GC = chassis->GetBody()->GetFrame_REF_to_abs() * ChFrame<>(rel_pos, rel_rot);

    // Create joint (DOFs about X and Y axes of X_GC frame)
    auto joint = chrono_types::make_shared<ChLinkUniversal>();
    joint->Initialize(body1, body2, X_GC);
    chassis->GetBody()->GetSystem()->AddLink(joint);
}

// Add a rotational speed motor between two bodies at the given position and orientation
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkMotorRotationSpeed> AddMotorSpeed(std::shared_ptr<ChBody> body1,
                                                        std::shared_ptr<ChBody> body2,
                                                        std::shared_ptr<ViperChassis> chassis,
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
                                                        std::shared_ptr<ViperChassis> chassis,
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
                                                          std::shared_ptr<ViperChassis> chassis,
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

// Add a spring between two bodies connected at the specified points
// (expressed in and relative to the chassis frame).
std::shared_ptr<ChLinkTSDA> AddSuspensionSpring(std::shared_ptr<ChBodyAuxRef> body1,
                                                std::shared_ptr<ChBodyAuxRef> body2,
                                                std::shared_ptr<ViperChassis> chassis,
                                                const ChVector<>& pos1,
                                                const ChVector<>& pos2) {
    const ChFrame<>& X_GP = chassis->GetBody()->GetFrame_REF_to_abs();
    auto p1 = X_GP.TransformPointLocalToParent(pos1);
    auto p2 = X_GP.TransformPointLocalToParent(pos2);

    std::shared_ptr<ChLinkTSDA> spring;
    spring = chrono_types::make_shared<ChLinkTSDA>();
    spring->Initialize(body1, body2, false, p1, p2);
    spring->SetSpringCoefficient(800000.0);
    spring->SetDampingCoefficient(10000.0);
    chassis->GetBody()->GetSystem()->AddLink(spring);
    return spring;
}

// =============================================================================

// Base class for all Viper Part
ViperPart::ViperPart(const std::string& name,
                     const ChFrame<>& rel_pos,
                     std::shared_ptr<ChMaterialSurface> mat,
                     bool collide)
    : m_name(name), m_pos(rel_pos), m_mat(mat), m_collide(collide), m_visualize(true) {}

void ViperPart::Construct(ChSystem* system) {
    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetNameString(m_name + "_body");
    m_body->SetMass(m_mass);
    m_body->SetInertiaXX(m_inertia);
    m_body->SetFrame_COG_to_REF(m_cog);

    // Add visualization shape
    if (m_visualize) {
        auto vis_mesh_file = GetChronoDataFile("robot/viper/obj/" + m_mesh_name + ".obj");
        auto trimesh_vis = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
        trimesh_vis->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName(m_mesh_name);
        trimesh_shape->SetMutable(false);
        m_body->AddVisualShape(trimesh_shape);
    }

    // Add collision shape
    if (m_collide) {
        auto col_mesh_file = GetChronoDataFile("robot/viper/col/" + m_mesh_name + ".obj");
        auto trimesh_col = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(col_mesh_file, false, false);
        trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
        trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

        m_body->GetCollisionModel()->ClearModel();
        m_body->GetCollisionModel()->AddTriangleMesh(m_mat, trimesh_col, false, false, VNULL, ChMatrix33<>(1), 0.005);
        m_body->GetCollisionModel()->BuildModel();
        m_body->SetCollide(m_collide);
    }

    system->AddBody(m_body);
}

void ViperPart::CalcMassProperties(double density) {
    auto mesh_filename = GetChronoDataFile("robot/viper/col/" + m_mesh_name + ".obj");
    auto trimesh_col = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(mesh_filename, false, false);
    trimesh_col->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
    trimesh_col->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

    double vol;
    ChVector<> cog_pos;
    ChMatrix33<> cog_rot;
    ChMatrix33<> inertia;
    trimesh_col->ComputeMassProperties(true, vol, cog_pos, inertia);
    ChInertiaUtils::PrincipalInertia(inertia, m_inertia, cog_rot);
    m_mass = density * vol;
    m_inertia *= density;
    m_cog = ChFrame<>(cog_pos, cog_rot);
}

void ViperPart::Initialize(std::shared_ptr<ChBodyAuxRef> chassis) {
    Construct(chassis->GetSystem());

    // Set absolute position
    ChFrame<> X_GC = chassis->GetFrame_REF_to_abs() * m_pos;
    m_body->SetFrame_REF_to_abs(X_GC);
}

// =============================================================================

// Rover Chassis
ViperChassis::ViperChassis(const std::string& name, std::shared_ptr<ChMaterialSurface> mat)
    : ViperPart(name, ChFrame<>(VNULL, QUNIT), mat, false) {
    m_mesh_name = "viper_chassis";
    m_color = ChColor(1.0f, 1.0f, 1.0f);
    CalcMassProperties(165);
}

void ViperChassis::Initialize(ChSystem* system, const ChFrame<>& pos) {
    Construct(system);

    m_body->SetFrame_REF_to_abs(pos);
}

// =============================================================================

// Viper Wheel
ViperWheel::ViperWheel(const std::string& name,
                       const ChFrame<>& rel_pos,
                       std::shared_ptr<ChMaterialSurface> mat,
                       ViperWheelType wheel_type)
    : ViperPart(name, rel_pos, mat, true) {
    switch (wheel_type) {
        case ViperWheelType::RealWheel:
            m_mesh_name = "viper_wheel";
            break;
        case ViperWheelType::SimpleWheel:
            m_mesh_name = "viper_simplewheel";
            break;
        case ViperWheelType::CylWheel:
            m_mesh_name = "viper_cylwheel";
            break;
    }

    m_color = ChColor(0.4f, 0.7f, 0.4f);
    CalcMassProperties(800);
}

// =============================================================================

// Viper Upper Suspension Arm
ViperUpperArm::ViperUpperArm(const std::string& name,
                             const ChFrame<>& rel_pos,
                             std::shared_ptr<ChMaterialSurface> mat,
                             const int& side)
    : ViperPart(name, rel_pos, mat, false) {
    if (side == 0) {
        m_mesh_name = "viper_L_up_sus";
    } else if (side == 1) {
        m_mesh_name = "viper_R_up_sus";
    }

    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(2000);
}

// =============================================================================

// Viper Lower Suspension Arm
ViperLowerArm::ViperLowerArm(const std::string& name,
                             const ChFrame<>& rel_pos,
                             std::shared_ptr<ChMaterialSurface> mat,
                             const int& side)
    : ViperPart(name, rel_pos, mat, false) {
    if (side == 0) {
        m_mesh_name = "viper_L_bt_sus";
    } else if (side == 1) {
        m_mesh_name = "viper_R_bt_sus";
    }

    m_color = ChColor(0.7f, 0.4f, 0.4f);
    CalcMassProperties(4500);
}

// =============================================================================

// Viper Upright
ViperUpright::ViperUpright(const std::string& name,
                           const ChFrame<>& rel_pos,
                           std::shared_ptr<ChMaterialSurface> mat,
                           const int& side)
    : ViperPart(name, rel_pos, mat, false) {
    if (side == 0) {
        m_mesh_name = "viper_L_steer";
    } else if (side == 1) {
        m_mesh_name = "viper_R_steer";
    }

    m_color = ChColor(0.7f, 0.7f, 0.7f);
    CalcMassProperties(4500);
}

// =============================================================================

// Rover model
Viper::Viper(ChSystem* system, ViperWheelType wheel_type) : m_system(system), m_chassis_fixed(false) {
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

    Create(wheel_type);
}

void Viper::Create(ViperWheelType wheel_type) {
    // create rover chassis
    m_chassis = chrono_types::make_shared<ViperChassis>("chassis", m_default_material);

    // initilize rover wheels
    double wx = 0.5618 + 0.08;
    double wy = 0.2067 + 0.32 + 0.0831;
    double wz = 0.0;

    m_wheels[V_LF] = chrono_types::make_shared<ViperWheel>("wheel_LF", ChFrame<>(ChVector<>(+wx, +wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);
    m_wheels[V_RF] = chrono_types::make_shared<ViperWheel>("wheel_RF", ChFrame<>(ChVector<>(+wx, -wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);
    m_wheels[V_LB] = chrono_types::make_shared<ViperWheel>("wheel_LB", ChFrame<>(ChVector<>(-wx, +wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);
    m_wheels[V_RB] = chrono_types::make_shared<ViperWheel>("wheel_RB", ChFrame<>(ChVector<>(-wx, -wy, wz), QUNIT),
                                                           m_wheel_material, wheel_type);

    m_wheels[V_LF]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));
    m_wheels[V_LB]->m_mesh_xform = ChFrame<>(VNULL, Q_from_AngZ(CH_C_PI));

    // create rover upper and lower suspension arms
    double cr_lx = 0.5618 + 0.08;
    double cr_ly = 0.2067;  // + 0.32/2;
    double cr_lz = 0.0525;

    ChVector<> cr_rel_pos_lower[] = {
        ChVector<>(+cr_lx, +cr_ly, -cr_lz),  // LF
        ChVector<>(+cr_lx, -cr_ly, -cr_lz),  // RF
        ChVector<>(-cr_lx, +cr_ly, -cr_lz),  // LB
        ChVector<>(-cr_lx, -cr_ly, -cr_lz)   // RB
    };

    ChVector<> cr_rel_pos_upper[] = {
        ChVector<>(+cr_lx, +cr_ly, cr_lz),  // LF
        ChVector<>(+cr_lx, -cr_ly, cr_lz),  // RF
        ChVector<>(-cr_lx, +cr_ly, cr_lz),  // LB
        ChVector<>(-cr_lx, -cr_ly, cr_lz)   // RB
    };

    for (int i = 0; i < 4; i++) {
        m_lower_arms[i] = chrono_types::make_shared<ViperLowerArm>("lower_arm", ChFrame<>(cr_rel_pos_lower[i], QUNIT),
                                                                   m_default_material, i % 2);
        m_upper_arms[i] = chrono_types::make_shared<ViperUpperArm>("upper_arm", ChFrame<>(cr_rel_pos_upper[i], QUNIT),
                                                                   m_default_material, i % 2);
    }

    // create uprights
    double sr_lx = 0.5618 + 0.08;
    double sr_ly = 0.2067 + 0.32 + 0.0831;
    double sr_lz = 0.0;
    ChVector<> sr_rel_pos[] = {
        ChVector<>(+sr_lx, +sr_ly, -sr_lz),  // LF
        ChVector<>(+sr_lx, -sr_ly, -sr_lz),  // RF
        ChVector<>(-sr_lx, +sr_ly, -sr_lz),  // LB
        ChVector<>(-sr_lx, -sr_ly, -sr_lz)   // RB
    };

    for (int i = 0; i < 4; i++) {
        m_uprights[i] = chrono_types::make_shared<ViperUpright>("upright", ChFrame<>(sr_rel_pos[i], QUNIT),
                                                                m_default_material, i % 2);
    }

    // create drive shafts
    for (int i = 0; i < 4; i++) {
        m_drive_shafts[i] = chrono_types::make_shared<ChShaft>();
    }
}

void Viper::Initialize(const ChFrame<>& pos) {
    assert(m_driver);

    m_chassis->Initialize(m_system, pos);
    m_chassis->GetBody()->SetBodyFixed(m_chassis_fixed);

    for (int i = 0; i < 4; i++) {
        m_wheels[i]->Initialize(m_chassis->GetBody());
        m_upper_arms[i]->Initialize(m_chassis->GetBody());
        m_lower_arms[i]->Initialize(m_chassis->GetBody());
        m_uprights[i]->Initialize(m_chassis->GetBody());
    }

    // add all constraints to the system
    // redefine pos data for constraints
    double sr_lx = 0.5618 + 0.08;
    // double sr_ly = 0.2067 + 0.32 + 0.0831;
    // double sr_lz = 0.0;
    double sr_ly_joint = 0.2067 + 0.32;

    double cr_lx = 0.5618 + 0.08;
    double cr_ly = 0.2067;  // + 0.32/2;
    double cr_lz = 0.0525;

    double w_lx = 0.5618 + 0.08;
    double w_ly = 0.2067 + 0.32 + 0.0831;
    double w_lz = 0.0;

    ChVector<> wheel_rel_pos[] = {
        ChVector<>(+w_lx, +w_ly, w_lz),  // LF
        ChVector<>(+w_lx, -w_ly, w_lz),  // RF
        ChVector<>(-w_lx, +w_ly, w_lz),  // LB
        ChVector<>(-w_lx, -w_ly, w_lz)   // RB
    };

    ChVector<> sr_rel_pos_lower[] = {
        ChVector<>(+sr_lx, +sr_ly_joint, -cr_lz),  // LF
        ChVector<>(+sr_lx, -sr_ly_joint, -cr_lz),  // RF
        ChVector<>(-sr_lx, +sr_ly_joint, -cr_lz),  // LB
        ChVector<>(-sr_lx, -sr_ly_joint, -cr_lz)   // RB
    };

    ChVector<> sr_rel_pos_upper[] = {
        ChVector<>(+sr_lx, +sr_ly_joint, cr_lz),  // LF
        ChVector<>(+sr_lx, -sr_ly_joint, cr_lz),  // RF
        ChVector<>(-sr_lx, +sr_ly_joint, cr_lz),  // LB
        ChVector<>(-sr_lx, -sr_ly_joint, cr_lz)   // RB
    };

    ChVector<> cr_rel_pos_lower[] = {
        ChVector<>(+cr_lx, +cr_ly, -cr_lz),  // LF
        ChVector<>(+cr_lx, -cr_ly, -cr_lz),  // RF
        ChVector<>(-cr_lx, +cr_ly, -cr_lz),  // LB
        ChVector<>(-cr_lx, -cr_ly, -cr_lz)   // RB
    };

    ChVector<> cr_rel_pos_upper[] = {
        ChVector<>(+cr_lx, +cr_ly, cr_lz),  // LF
        ChVector<>(+cr_lx, -cr_ly, cr_lz),  // RF
        ChVector<>(-cr_lx, +cr_ly, cr_lz),  // LB
        ChVector<>(-cr_lx, -cr_ly, cr_lz)   // RB
    };

    // Orientation of steer motors.
    // A positive steering input results in positive (left) front wheel steering and negative (right) rear wheel
    // steering.
    ChQuaternion<> sm_rot[] = {
        QUNIT,                 // LF
        QUNIT,                 // RF
        Q_from_AngX(CH_C_PI),  // LB
        Q_from_AngX(CH_C_PI)   // RB
    };

    // Orientation of lift motors.
    // A positive lifting input results in rasing the chassis relative to the wheels.
    ChQuaternion<> lm_rot[] = {
        QUNIT,                 // LF
        Q_from_AngX(CH_C_PI),  // RF
        QUNIT,                 // LB
        Q_from_AngX(CH_C_PI)   // RB
    };

    ChQuaternion<> z2x = Q_from_AngY(CH_C_PI_2);

    for (int i = 0; i < 4; i++) {
        AddUniversalJoint(m_lower_arms[i]->GetBody(), m_uprights[i]->GetBody(), m_chassis, sr_rel_pos_lower[i], QUNIT);
        AddUniversalJoint(m_upper_arms[i]->GetBody(), m_uprights[i]->GetBody(), m_chassis, sr_rel_pos_upper[i], QUNIT);

        // Add lifting motors at the connecting points between upper_arm & chassis and lower_arm & chassis
        m_lift_motor_funcs[i] = chrono_types::make_shared<ChFunction_Const>(0.0);
        m_lift_motors[i] = AddMotorAngle(m_chassis->GetBody(), m_lower_arms[i]->GetBody(), m_chassis,
                                         cr_rel_pos_lower[i], z2x * lm_rot[i]);
        m_lift_motors[i]->SetMotorFunction(m_lift_motor_funcs[i]);
        AddRevoluteJoint(m_chassis->GetBody(), m_upper_arms[i]->GetBody(), m_chassis, cr_rel_pos_upper[i], z2x);

        auto steer_rod = chrono_types::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 1000, true, false);
        steer_rod->SetPos(m_wheels[i]->GetPos());
        steer_rod->SetBodyFixed(false);
        m_system->Add(steer_rod);

        ChQuaternion<> z2y;
        z2y.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(1, 0, 0));

        switch (m_driver->GetDriveMotorType()) {
            case ViperDriver::DriveMotorType::SPEED:
                m_drive_motor_funcs[i] = chrono_types::make_shared<ChFunction_Setpoint>();
                m_drive_motors[i] = AddMotorSpeed(steer_rod, m_wheels[i]->GetBody(), m_chassis, wheel_rel_pos[i], z2y);
                m_drive_motors[i]->SetMotorFunction(m_drive_motor_funcs[i]);
                break;
            case ViperDriver::DriveMotorType::TORQUE:
                AddRevoluteJoint(steer_rod, m_wheels[i]->GetBody(), m_chassis, wheel_rel_pos[i], z2y);
                break;
        }

        m_steer_motor_funcs[i] = chrono_types::make_shared<ChFunction_Const>(0.0);
        m_steer_motors[i] = AddMotorAngle(steer_rod, m_uprights[i]->GetBody(), m_chassis, wheel_rel_pos[i], sm_rot[i]);
        m_steer_motors[i]->SetMotorFunction(m_steer_motor_funcs[i]);

        m_springs[i] = AddSuspensionSpring(m_chassis->GetBody(), m_uprights[i]->GetBody(), m_chassis,
                                           cr_rel_pos_upper[i], sr_rel_pos_lower[i]);
    }

    double J = 0.1;  // shaft rotational inertia
    for (int i = 0; i < 4; i++) {
        m_drive_shafts[i]->SetInertia(J);
        m_system->Add(m_drive_shafts[i]);

        // Connect shaft aligned with the wheel's axis of rotation (local wheel Y).
        // Set connection such that a positive torque applied to the shaft results in forward rover motion.
        auto shaftbody_connection = chrono_types::make_shared<ChShaftsBody>();
        shaftbody_connection->Initialize(m_drive_shafts[i], m_wheels[i]->GetBody(), ChVector<>(0, 0, -1));
        m_system->Add(shaftbody_connection);
    }
}

void Viper::SetDriver(std::shared_ptr<ViperDriver> driver) {
    m_driver = driver;
    m_driver->viper = this;
}

void Viper::SetWheelContactMaterial(std::shared_ptr<ChMaterialSurface> mat) {
    for (auto& wheel : m_wheels)
        wheel->m_mat = mat;
}

void Viper::SetChassisFixed(bool fixed) {
    m_chassis_fixed = fixed;
}

void Viper::SetChassisVisualization(bool state) {
    m_chassis->SetVisualize(state);
}

void Viper::SetWheelVisualization(bool state) {
    for (auto& wheel : m_wheels)
        wheel->SetVisualize(state);
}

void Viper::SetSuspensionVisualization(bool state) {
    for (auto& p : m_lower_arms)
        p->SetVisualize(state);
    for (auto& p : m_upper_arms)
        p->SetVisualize(state);
    for (auto& p : m_uprights)
        p->SetVisualize(state);
}

ChVector<> Viper::GetWheelContactForce(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactForce();
}

ChVector<> Viper::GetWheelContactTorque(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetContactTorque();
}

ChVector<> Viper::GetWheelAppliedForce(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedForce();
}

ChVector<> Viper::GetWheelAppliedTorque(ViperWheelID id) const {
    return m_wheels[id]->GetBody()->GetAppliedTorque();
}

double Viper::GetWheelTracTorque(ViperWheelID id) const {
    if (m_driver->GetDriveMotorType() == ViperDriver::DriveMotorType::TORQUE)
        return 0;

    return m_drive_motors[id]->GetMotorTorque();
}

double Viper::GetRoverMass() const {
    double tot_mass = m_chassis->GetBody()->GetMass();
    for (int i = 0; i < 4; i++) {
        tot_mass += m_wheels[i]->GetBody()->GetMass();
        tot_mass += m_upper_arms[i]->GetBody()->GetMass();
        tot_mass += m_lower_arms[i]->GetBody()->GetMass();
        tot_mass += m_uprights[i]->GetBody()->GetMass();
    }
    return tot_mass;
}

double Viper::GetWheelMass() const {
    return m_wheels[0]->GetBody()->GetMass();
}

void Viper::Update() {
    double time = m_system->GetChTime();
    m_driver->Update(time);

    for (int i = 0; i < 4; i++) {
        // Extract driver inputs
        double driving = m_driver->drive_speeds[i];
        double steering = m_driver->steer_angles[i];
        double lifting = m_driver->lift_angles[i];

        // Enforce maximum steering angle
        ChClampValue(steering, -m_max_steer_angle, +m_max_steer_angle);

        // Set motor functions
        m_steer_motor_funcs[i]->Set_yconst(steering);
        m_lift_motor_funcs[i]->Set_yconst(lifting);
        if (m_driver->GetDriveMotorType() == ViperDriver::DriveMotorType::SPEED)
            m_drive_motor_funcs[i]->SetSetpoint(driving, time);
    }
}

// =============================================================================

ViperDriver::ViperDriver()
    : drive_speeds({0, 0, 0, 0}), steer_angles({0, 0, 0, 0}), lift_angles({0, 0, 0, 0}), viper(nullptr) {}

void ViperDriver::SetSteering(double angle) {
    for (int i = 0; i < 4; i++)
        steer_angles[i] = angle;
}

void ViperDriver::SetSteering(double angle, ViperWheelID id) {
    steer_angles[id] = angle;
}

/// Set current lift input angles.
void ViperDriver::SetLifting(double angle) {
    for (int i = 0; i < 4; i++)
        lift_angles[i] = angle;
}

ViperDCMotorControl::ViperDCMotorControl()
    : m_stall_torque({300, 300, 300, 300}), m_no_load_speed({CH_C_PI, CH_C_PI, CH_C_PI, CH_C_PI}) {}

void ViperDCMotorControl::Update(double time) {
    double speed_reading;
    double target_torque;
    for (int i = 0; i < 4; i++) {
        speed_reading = -viper->m_drive_shafts[i]->GetPos_dt();

        if (speed_reading > m_no_load_speed[i]) {
            target_torque = 0;
        } else if (speed_reading < 0) {
            target_torque = m_stall_torque[i];
        } else {
            target_torque = m_stall_torque[i] * ((m_no_load_speed[i] - speed_reading) / m_no_load_speed[i]);
        }

        viper->m_drive_shafts[i]->SetAppliedTorque(-target_torque);
    }
}

ViperSpeedDriver::ViperSpeedDriver(double time_ramp, double speed) : m_ramp(time_ramp), m_speed(speed) {}

void ViperSpeedDriver::Update(double time) {
    double speed = m_speed;
    if (time < m_ramp)
        speed = m_speed * (time / m_ramp);
    drive_speeds = {speed, speed, speed, speed};
}

}  // namespace viper
}  // namespace chrono
