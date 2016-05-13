// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Justin Madsen
// =============================================================================
//
// model a single track chain system, as part of a tracked vehicle.
// Static variable values are based on a M113 model in the report by Shabana
//
// =============================================================================

#include <cstdio>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "motion_functions/ChFunction.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/suspension/TorsionArmSuspension.h"

namespace chrono {

TorsionArmSuspension::TorsionArmSuspension(const std::string& name,
                                           VisualizationType::Enum vis,
                                           CollisionType::Enum collide,
                                           size_t chainSys_idx,
                                           double wheelMass,
                                           const ChVector<>& wheelIxx,
                                           double armMass,
                                           const ChVector<>& armIxx,
                                           double springK,
                                           double springC,
                                           double springPreload,
                                           double mu,
                                           double wheel_width,
                                           double wheel_width_gap,
                                           double wheel_radius,
                                           ChVector<> wheel_pos_rel,
                                           double arm_radius,
                                           bool use_custom_spring)
    : m_vis(vis),
      m_collide(collide),
      m_chainSys_idx(chainSys_idx),
      m_wheelMass(wheelMass),
      m_wheelInertia(wheelIxx),
      m_armMass(armMass),
      m_armInertia(armIxx),
      m_springK(springK),
      m_springC(springC),
      m_TorquePreload(springPreload),
      m_mu(mu),
      m_wheelWidth(wheel_width),
      m_wheelWidthGap(wheel_width_gap),
      m_wheelRadius(wheel_radius),
      m_wheel_PosRel(wheel_pos_rel),
      m_armRadius(arm_radius),
      m_use_custom_spring(use_custom_spring),
      m_meshFile(vehicle::GetDataFile("M113/Roller_XforwardYup.obj")),
      m_meshName("Road wheel") {
    // FILE* fp = fopen(filename.c_str(), "r");
    // char readBuffer[65536];
    // fclose(fp);

    Create(name);
}

// 1) load data from file, 2) create bodies using data
void TorsionArmSuspension::Create(const std::string& name) {

    // create the suspension arm body
    m_arm = ChSharedPtr<ChBody>(new ChBody);
    m_arm->SetNameString(name + "_arm");
    m_arm->SetMass(m_armMass);
    m_arm->SetInertiaXX(m_armInertia);  // link distance along y-axis
    // create the roadwheel body
    m_wheel = ChSharedPtr<ChBody>(new ChBody);
    m_wheel->SetNameString(name + "_roadWheel");
    m_wheel->SetMass(m_wheelMass);
    m_wheel->SetInertiaXX(m_wheelInertia);  // wheel width along z-axis

    // relative distance from chassis/arm pin to wheel center

    // create the constraints
    m_armChassis_rev = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_armChassis_rev->SetName("_arm-chassis_revolute");
    m_armWheel_rev = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_armWheel_rev->SetName("_arm-wheel-revolute");

    // create the torsional spring damper assembly
    m_rot_spring = new ChLinkForce;
    m_rot_spring->Set_active(1);
    m_rot_spring->Set_K(m_springK);
    m_rot_spring->Set_R(m_springC);
    m_rot_spring->Set_iforce(m_TorquePreload);

    // if we're using a custom spring...
    if (m_use_custom_spring) {
        m_custom_spring = ChSharedPtr<ChFunction_CustomSpring>(new ChFunction_CustomSpring);
        m_rot_spring->Set_K(1);
        m_rot_spring->Set_modul_K(m_custom_spring.get_ptr());
    }
    // add the link force to the rev joint
    m_armChassis_rev->SetForce_Rz(m_rot_spring);

    // add visualization assets to the roadwheel and arm
    AddVisualization();
}

// local_Csys is at the pin location between arm and chassis.
// z-axis defines axis of revolute joint rotation.
// Wheel rot is the same as local_Csys.
void TorsionArmSuspension::Initialize(ChSharedPtr<ChBody> chassis,
                                      const ChFrame<>& chassis_REF,
                                      const ChCoordsys<>& local_Csys) {
    // add collision geometry, for the wheel
    (local_Csys.pos.z < 0) ? AddCollisionGeometry(LEFTSIDE, m_mu, 0.9 * m_mu, 0.2)
                           : AddCollisionGeometry(RIGHTSIDE, m_mu, 0.9 * m_mu, 0.2);

    // correct armpin to wheel distance for left/right sides
    if (local_Csys.pos.z < 0) {
        m_wheel_PosRel.z *= -1;
    }

    // Express the revolute joint location in the absolute coordinate system.
    ChFrame<> pin1_abs(local_Csys);
    pin1_abs.ConcatenatePreTransformation(chassis_REF);

    // wheel COM frame, absoluate c-sys
    ChFrame<> wheel_COG_abs(local_Csys.pos + GetWheelPosRel(), local_Csys.rot);
    wheel_COG_abs.ConcatenatePreTransformation(chassis_REF);

    // arm COG is between the two pin locastions on the arm
    ChFrame<> pin2_abs(local_Csys);
    // second pin on arm is not offset in the lateral direction
    ChVector<> arm_rel(GetWheelPosRel());
    arm_rel.z = 0;
    pin2_abs.SetPos(local_Csys.pos + arm_rel);
    pin2_abs.ConcatenatePreTransformation(chassis_REF);

    m_arm->SetPos((pin1_abs.GetPos() + pin2_abs.GetPos()) / 2.0);

    // y-axis should point along length of arm, according to inertia tensor
    ChVector<> v = (pin2_abs.GetPos() - pin1_abs.GetPos()).GetNormalized();
    // use the z-axis from the wheel frame
    ChVector<> w = (pin1_abs.GetRot().GetZaxis()).GetNormalized();
    ChVector<> u = Vcross(v, w);
    u.Normalize();
    ChMatrix33<> rot;
    // z-axis of the wheel might not be exactly orthogonal to arm y-axis
    rot.Set_A_axis(u, v, Vcross(u, v));
    // should give the correct orientation to the arm
    m_arm->SetRot(rot);
    // pos, rot of arm set, add it to the system
    chassis->GetSystem()->Add(m_arm);

    // set the wheel in the correct position.
    m_wheel->SetPos(wheel_COG_abs.GetPos());
    // inertia and visual assets have wheel width along z-axis locally.
    // No need to rotate the wheel.
    m_wheel->SetRot(wheel_COG_abs.GetRot());
    chassis->GetSystem()->Add(m_wheel);

    // init and add the revolute joints
    // arm-chassis, z-axis is already in the lateral direction
    m_armChassis_rev->Initialize(m_arm, chassis, ChCoordsys<>(pin1_abs.GetPos(), pin1_abs.GetRot()));
    // init and finish setting up the torsional spring, since it's part of this revolute constraint
    chassis->GetSystem()->AddLink(m_armChassis_rev);

    // wheel-arm, z-axis is already in the lateral direction
    ChVector<> rev_pos_abs = (wheel_COG_abs.GetPos() - pin2_abs.GetPos()) / 2.0 + pin2_abs.GetPos();
    m_armWheel_rev->Initialize(m_wheel, m_arm, ChCoordsys<>(rev_pos_abs, wheel_COG_abs.GetRot()));
    chassis->GetSystem()->AddLink(m_armWheel_rev);
}

/// add a cylinder to model the torsion bar arm and the wheel
void TorsionArmSuspension::AddVisualization() {
    // add visualization assets
    switch (m_vis) {
        case VisualizationType::Primitives: {
            // define the wheel as two concentric cylinders with a gap
            ChSharedPtr<ChCylinderShape> cylA(new ChCylinderShape);
            cylA->GetCylinderGeometry().p1 = ChVector<>(0, 0, m_wheelWidth / 2.0);
            cylA->GetCylinderGeometry().p2 = ChVector<>(0, 0, m_wheelWidthGap / 2.0);
            cylA->GetCylinderGeometry().rad = m_wheelRadius;
            m_wheel->AddAsset(cylA);

            // second cylinder is a mirror of the first
            ChSharedPtr<ChCylinderShape> cylB(new ChCylinderShape);
            cylB->GetCylinderGeometry().p1 = ChVector<>(0, 0, -m_wheelWidth / 2.0);
            cylB->GetCylinderGeometry().p2 = ChVector<>(0, 0, -m_wheelWidthGap / 2.0);
            cylB->GetCylinderGeometry().rad = m_wheelRadius;
            m_wheel->AddAsset(cylB);

            // put a texture on the wheel
            ChSharedPtr<ChTexture> tex(new ChTexture);
            tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            m_wheel->AddAsset(tex);

            // define the arm cylinder shape, link is along the y-axis
            ChVector<> wheel_posRel_XY = m_wheel_PosRel;
            wheel_posRel_XY.z = 0;
            // arm length will be the x-y dims
            double armLength = wheel_posRel_XY.Length();
            ChVector<> p1_arm(0, armLength / 2.0, 0);
            ChVector<> p2_arm(0, -armLength / 2.0, 0);
            ChSharedPtr<ChCylinderShape> arm_cyl(new ChCylinderShape);
            arm_cyl->GetCylinderGeometry().p1 = p1_arm;
            arm_cyl->GetCylinderGeometry().p2 = p2_arm;
            arm_cyl->GetCylinderGeometry().rad = m_armRadius;
            m_arm->AddAsset(arm_cyl);
            break;
        }
        case VisualizationType::Mesh: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(getMeshName());
            m_wheel->AddAsset(trimesh_shape);

            ChSharedPtr<ChTexture> mcolor(new ChTexture);
            mcolor->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            // ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.2f, 0.2f, 0.2f));
            m_wheel->AddAsset(mcolor);

            // add the linkage arm
            // define the arm cylinder shape, link is along the y-axis
            ChVector<> wheel_posRel_XY = m_wheel_PosRel;
            wheel_posRel_XY.z = 0;
            // arm length will be the x-y dims
            double armLength = wheel_posRel_XY.Length();
            ChVector<> p1_arm(0, armLength / 2.0, 0);
            ChVector<> p2_arm(0, -armLength / 2.0, 0);
            ChSharedPtr<ChCylinderShape> arm_cyl(new ChCylinderShape);
            arm_cyl->GetCylinderGeometry().p1 = p1_arm;
            arm_cyl->GetCylinderGeometry().p2 = p2_arm;
            arm_cyl->GetCylinderGeometry().rad = m_armRadius;
            m_arm->AddAsset(arm_cyl);

            break;
        }
        default: { GetLog() << "Didn't recognize VisualizationType for TorsionarmSuspension \n"; }
    }  // end switch
}

/// only the road wheels are used for collision
void TorsionArmSuspension::AddCollisionGeometry(VehicleSide side,
                                                double mu,
                                                double mu_sliding,
                                                double mu_roll,
                                                double mu_spin) {
    // add collision geometrey, if enabled. Warn if not
    if (m_collide == CollisionType::None) {
        m_wheel->SetCollide(false);
        GetLog() << " !!! Road Wheel " << m_wheel->GetName() << " collision deactivated !!! \n\n";
        return;
    }

    m_wheel->SetCollide(true);
    m_wheel->GetCollisionModel()->ClearModel();

    m_wheel->GetCollisionModel()->SetSafeMargin(0.002);  // inward safe margin
    m_wheel->GetCollisionModel()->SetEnvelope(0.008);    // distance of the outward "collision envelope"

    // set the collision material
    m_wheel->GetMaterialSurface()->SetSfriction(mu);
    m_wheel->GetMaterialSurface()->SetKfriction(mu_sliding);
    m_wheel->GetMaterialSurface()->SetRollingFriction(mu_roll);
    m_wheel->GetMaterialSurface()->SetSpinningFriction(mu_spin);
    // m_wheel->GetMaterialSurface()->SetCompliance(0.00002);

    switch (m_collide) {
        case CollisionType::Primitives: {
            double cyl_width = 0.5 * (m_wheelWidth - m_wheelWidthGap);
            ChVector<> shape_offset = ChVector<>(0, 0, 0.5 * (cyl_width + m_wheelWidthGap));
            // use two simple cylinders. Default is along the y-axis, here we have z-axis relative to chassis.
            m_wheel->GetCollisionModel()->AddCylinder(m_wheelRadius, m_wheelRadius, 0.5 * cyl_width, shape_offset,
                                                      Q_from_AngAxis(CH_C_PI_2, VECT_X));

            // mirror first cylinder about the x-y plane
            shape_offset.z *= -1;
            m_wheel->GetCollisionModel()->AddCylinder(m_wheelRadius, m_wheelRadius, 0.5 * cyl_width, shape_offset,
                                                      Q_from_AngAxis(CH_C_PI_2, VECT_X));
        }
        case CollisionType::Mesh: {
            // use a triangle mesh
            /*

                geometry::ChTriangleMeshSoup temp_trianglemesh;

            // TODO: fill the triangleMesh here with some track shoe geometry

            // is there an offset??
            double shoelength = 0.2;
            ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
            m_wheel->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);
            */

            break;
        }
        case CollisionType::ConvexHull: {
            /*
            // use convex hulls, loaded from file
            ChStreamInAsciiFile chull_file(GetChronoDataFile("drive_gear.chulls").c_str());
            // transform the collision geometry as needed
            double mangle = 45.0; // guess
            ChQuaternion<>rot;
            rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
            ChMatrix33<> rot_offset(rot);
            ChVector<> disp_offset(0,0,0);  // no displacement offset
            m_wheel->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
            */

            break;
        }
        default:
            // no collision geometry
            GetLog() << "not recognized CollisionType: " << (int)m_collide << " for road wheel \n";
            m_wheel->SetCollide(false);
            return;
    }  // end switch

    // setup collision family, road wheel is a rolling element
    m_wheel->GetCollisionModel()->SetFamily((int)CollisionFam::Wheel);

    // only collide w/ shoes on the same side of the vehicle
    if (side == RIGHTSIDE) {
        m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeLeft);
    } else {
        m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeRight);
    }

    // don't collide with the other rolling elements
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Wheel);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Hull);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Gear);
    m_wheel->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Ground);

    m_wheel->GetCollisionModel()->BuildModel();
}

/// write headers for the output data file to the input ostream
void TorsionArmSuspension::Write_header(const std::string& filename, DebugType type) {
    if (type & DBG_BODY) {
        m_filename_DBG_BODY = filename;
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str());
        // headers
        ofile << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz\n";
    }
    if (type & DBG_CONTACTS) {
    }
    if (type & DBG_CONSTRAINTS) {
        m_filename_DBG_CV = filename;
        ChStreamOutAsciiFile ofile(filename.c_str());
        // headers
        ofile << "time,x1,y1,z1,rx1,ry1,x2,y2,z2,rx2,ry2\n";
    }
}

/// write constraint violation of wheel rev. constraint
void TorsionArmSuspension::Write_data(double t, DebugType type) {
    if (type & DBG_BODY) {
        std::stringstream ss;
        ChSharedPtr<ChBody> body = GetWheelBody();
        // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz
        ss << t << "," << body->GetPos() << "," << body->GetPos_dt() << "," << body->GetWvel_loc() << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str(), std::ios::app);
        ofile << ss.str().c_str();
    }
    if (type & DBG_CONTACTS) {
        // todo
    }
    if (type & DBG_CONSTRAINTS) {
        std::stringstream ss;
        ChMatrix<>* C = m_armChassis_rev->GetC();
        for (int row = 0; row < C->GetRows(); row++) {
            ss << "," << C->GetElement(row, 0);
        }

        C = m_armWheel_rev->GetC();
        for (int row = 0; row < C->GetRows(); row++) {
            ss << "," << C->GetElement(row, 0);
        }
        ss << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_CV.c_str(), std::ios::app);
        ofile << ss.str().c_str();
    }
}

}  // end namespace chrono
