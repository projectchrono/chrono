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
// The drive gear propels the tracked vehicle.
//	Body is driven with a specified rotational motion.
//
// =============================================================================

#include "DriveGearMotion.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"

#include "geometry/ChTriangleMeshSoup.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/collision/CollisionReporters.h"

namespace chrono {

// static variables
// for primitive collision/visualization
// const double DriveGearMotion::m_radius = 0.212; // to collision surface
// const double DriveGearMotion::m_width = 0.34;
// const double DriveGearMotion::m_widthGap = 0.189; // 0.189; // inner distance between cydliners

DriveGearMotion::DriveGearMotion(const std::string& name,
                                 VisualizationType::Enum vis,
                                 CollisionType::Enum collide,
                                 size_t chainSys_idx,
                                 double mass,
                                 const ChVector<>& gear_Ixx,
                                 double mu,
                                 double max_gear_omega)
    : m_vis(vis),
      m_collide(collide),
      m_meshFile(vehicle::GetDataFile("M113/Sprocket_XforwardYup.obj")),
      m_chainSys_idx(chainSys_idx),
      m_mass(mass),
      m_inertia(gear_Ixx),
      m_mu(mu),
      m_maxOmega(max_gear_omega),
      m_meshName("gear_mesh"),
      m_gearPinGeom(ChSharedPtr<GearPinGeometry>(new GearPinGeometry())),
      m_radius(m_gearPinGeom->gear_base_radius),
      m_width(m_gearPinGeom->gear_seat_width),
      m_widthGap(m_gearPinGeom->gear_seat_width_max - m_gearPinGeom->gear_seat_width_min) {
    // create the body, set the basic info
    m_gear = ChSharedPtr<ChBody>(new ChBody);
    m_gear->SetNameString(name + "body");
    m_gear->SetMass(m_mass);
    m_gear->SetInertiaXX(m_inertia);

    // create the revolute joint
    m_revolute = ChSharedPtr<ChLinkEngine>(new ChLinkEngine);
    m_revolute->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    m_revolute->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_LOCK);
    m_revolute->SetNameString(name + "_revoluteEngine");

    // create the visuals
    AddVisualization();
}

DriveGearMotion::~DriveGearMotion() {
}

void DriveGearMotion::Initialize(ChSharedPtr<ChBody> chassis,
                                 const ChFrame<>& chassis_REF,
                                 const ChCoordsys<>& local_Csys,
                                 const std::vector<ChSharedPtr<ChBody> >& shoes,
                                 ChTrackVehicle* vehicle) {
    assert(shoes.size() > 0);

    // add any collision geometry
    VehicleSide chassis_side = RIGHTSIDE;
    if (local_Csys.pos.z < 0)
        chassis_side = LEFTSIDE;

    AddCollisionGeometry(shoes, vehicle, chassis_side, m_mu, 0.9*m_mu);

    // get the local frame in the absolute ref. frame
    ChFrame<> gear_to_abs(local_Csys);
    gear_to_abs.ConcatenatePreTransformation(chassis_REF);

    // transform the drive gear body, add to system
    m_gear->SetPos(gear_to_abs.GetPos());
    m_gear->SetRot(gear_to_abs.GetRot());
    chassis->GetSystem()->Add(m_gear);

    // initialize the revolute joint, add to system
    m_revolute->Initialize(chassis, m_gear, ChCoordsys<>(gear_to_abs.GetPos(), gear_to_abs.GetRot()));
    chassis->GetSystem()->AddLink(m_revolute);
}

void DriveGearMotion::Update(double time, double omega_throttle) {
    double throttle = clamp(omega_throttle);  // default range [-1,1]
    if (ChSharedPtr<ChFunction_Const> func = m_revolute->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
        func->Set_yconst(getMaxOmega() * throttle);
}

double DriveGearMotion::GetGearMotion() const {
    if (ChSharedPtr<ChFunction_Const> func = m_revolute->Get_spe_funct().DynamicCastTo<ChFunction_Const>())
        return func->Get_yconst();
    else
        return 0;
}

void DriveGearMotion::AddVisualization() {
    // Attach visualization asset
    switch (m_vis) {
        case VisualizationType::Primitives: {
            // matches the primitive found in collisionType collisionCallback
            // cylinder and a bunch of boxes
            ChSharedPtr<ChAssetLevel> boxLevel(new ChAssetLevel);
            ChSharedPtr<ChAssetLevel> cylLevel(new ChAssetLevel);

            // two cylinders for the base circle of the gear
            ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
            cyl->GetCylinderGeometry().rad = m_gearPinGeom->gear_base_radius;
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, m_gearPinGeom->gear_seat_width_min / 2.0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, m_gearPinGeom->gear_seat_width_max / 2.0);
            cylLevel->AddAsset(cyl);

            // second cylinder is a mirror of the first, about x-y plane
            ChSharedPtr<ChCylinderShape> cylB(new ChCylinderShape(*cyl.get_ptr()));
            cylB->GetCylinderGeometry().p1.z *= -1;
            cylB->GetCylinderGeometry().p2.z *= -1;
            cylLevel->AddAsset(cylB);

            ChSharedPtr<ChTexture> tex(new ChTexture);
            tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            cylLevel->AddAsset(tex);

            // all assets for the cylinder level are set by now
            m_gear->AddAsset(cylLevel);

            double init_rot =
                CH_C_PI /
                m_gearPinGeom->num_teeth;  // std::atan(0.079815/0.24719); // from sprocket geometry blender file
            for (size_t b_idx = 0; b_idx < m_gearPinGeom->num_teeth; b_idx++) {
                // this is the angle from the vertical (y-axis local y c-sys).
                double rot_ang = init_rot + 2.0 * init_rot * b_idx;
                // distance center of tooth is from the gear spin axis
                double len_from_rotaxis =
                    ChVector<>(m_gearPinGeom->tooth_mid_bar.x, m_gearPinGeom->tooth_mid_bar.y, 0).Length();
                // shift the box vertically, midpoint of base should be at center of gear
                // if the rotation is relative to sprocket COG, then this will end up in the right place
                ChVector<> box_center(0.5 * len_from_rotaxis * std::sin(rot_ang),
                                      0.5 * len_from_rotaxis * std::cos(rot_ang), m_gearPinGeom->tooth_mid_bar.z);

                // z-axis is out of the page, to rotate clockwise negate the rotation angle.
                ChMatrix33<> box_rot_mat(Q_from_AngAxis(-rot_ang, VECT_Z));

                // create the box asset with the pos/rot specified
                ChSharedPtr<ChBoxShape> box(new ChBoxShape);
                box->GetBoxGeometry().SetLengths(
                    ChVector<>(m_gearPinGeom->tooth_len, len_from_rotaxis, m_gearPinGeom->tooth_width));
                box->GetBoxGeometry().Pos = box_center;
                box->GetBoxGeometry().Rot = box_rot_mat;  // assume the box is rotated about the parent body (gear)
                                                          // c-sys AFTER the position change is made
                boxLevel->AddAsset(box);

                // symmetric about XY
                ChSharedPtr<ChBoxShape> boxSym(new ChBoxShape(*box.get_ptr()));
                boxSym->GetBoxGeometry().Pos.z *= -1;
                boxLevel->AddAsset(boxSym);
            }

            // make the teeth red
            ChSharedPtr<ChColorAsset> red(new ChColorAsset(0.8f, 0.2f, 0.1f, 0.5f));
            boxLevel->AddAsset(red);

            // all assets on box level are added by now
            m_gear->AddAsset(boxLevel);
            break;
        }
        case VisualizationType::Mesh: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(getMeshFile(), true, false);

            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(getMeshName());
            m_gear->AddAsset(trimesh_shape);

            ChSharedPtr<ChTexture> tex(new ChTexture);
            tex->SetTextureFilename(GetChronoDataFile("redwhite.png"));
            m_gear->AddAsset(tex);

            break;
        }

        // moved this above to primitives
        // case VisualizationType::CompoundPrimitives:

        default: { GetLog() << "Didn't recognize VisualizationType for DriveGearMotion \n"; }
    }
}

void DriveGearMotion::AddCollisionGeometry(const std::vector<ChSharedPtr<ChBody> >& shoes,
                                           ChTrackVehicle* vehicle,
                                           VehicleSide side,
                                           double mu,
                                           double mu_sliding,
                                           double mu_roll,
                                           double mu_spin) {
    // add collision geometrey, if enabled. Warn if disabled
    if (m_collide == CollisionType::None) {
        m_gear->SetCollide(false);
        GetLog() << " !!! DriveGearMotion " << m_gear->GetName() << " collision deactivated !!! \n\n";
        return;
    }

    m_gear->SetCollide(true);
    m_gear->GetCollisionModel()->ClearModel();

    m_gear->GetCollisionModel()->SetSafeMargin(0.002);  // inward safe margin
    m_gear->GetCollisionModel()->SetEnvelope(0.004);    // distance of the outward "collision envelope"

    // set the collision material
    m_gear->GetMaterialSurface()->SetSfriction(mu);
    m_gear->GetMaterialSurface()->SetKfriction(mu_sliding);
    m_gear->GetMaterialSurface()->SetRollingFriction(mu_roll);
    m_gear->GetMaterialSurface()->SetSpinningFriction(mu_spin);

    switch (m_collide) {
        case CollisionType::Primitives: {
            // a set of boxes to represent the top-most flat face of the gear tooth
            // as the gear should be oriented initially with the tooth base directly
            // above the COG, each tooth box is rotated from the initial half rotation angle
            double init_rot =
                CH_C_PI /
                m_gearPinGeom->num_teeth;  // std::atan(0.07334/0.24929); // from sprocket geometry blender file
            for (size_t b_idx = 0; b_idx < m_gearPinGeom->num_teeth; b_idx++) {
                // this is the angle from the vertical (y-axis local y c-sys).
                double rot_ang = init_rot + 2.0 * init_rot * b_idx;
                // distance center of tooth is from the gear spin axis
                double len_from_rotaxis =
                    ChVector<>(m_gearPinGeom->tooth_mid_bar.x, m_gearPinGeom->tooth_mid_bar.y, 0).Length();
                // shift the box vertically, midpoint of base should be at center of gear
                // if the rotation is relative to sprocket COG, then this will end up in the right place
                ChVector<> box_center(0.5 * len_from_rotaxis * std::sin(rot_ang),  // 0
                                      0.5 * len_from_rotaxis * std::cos(rot_ang),  //  0.5*len_from_rotaxis
                                      m_gearPinGeom->tooth_mid_bar.z);

                // z-axis is out of the page, to rotate clockwise negate the rotation angle.
                ChMatrix33<> box_rot_mat(Q_from_AngAxis(-rot_ang, VECT_Z));

                m_gear->GetCollisionModel()->AddBox(
                    0.5 * m_gearPinGeom->tooth_len, 0.5 * len_from_rotaxis, 0.5 * m_gearPinGeom->tooth_width,
                    box_center,
                    box_rot_mat);  // does this rotation occur about gear c-sys or center of box ????

                // the gear teeth are symmetric about XY plane
                box_center.z *= -1;
                m_gear->GetCollisionModel()->AddBox(
                    0.5 * m_gearPinGeom->tooth_len, 0.5 * len_from_rotaxis, 0.5 * m_gearPinGeom->tooth_width,
                    box_center,
                    box_rot_mat);  // does this rotation occur about gear c-sys or center of box ????
            }

            ChVector<> shape_offset =
                ChVector<>(0, 0, 0.5 * (m_gearPinGeom->tooth_width + m_gearPinGeom->gear_seat_width_min));
            // use two simple cylinders.
            m_gear->GetCollisionModel()->AddCylinder(m_gearPinGeom->gear_base_radius, m_gearPinGeom->gear_base_radius,
                                                     0.5 * m_gearPinGeom->tooth_width, shape_offset,
                                                     Q_from_AngAxis(CH_C_PI_2, VECT_X));

            // mirror first cylinder about the x-y plane
            shape_offset.z *= -1;
            m_gear->GetCollisionModel()->AddCylinder(m_gearPinGeom->gear_base_radius, m_gearPinGeom->gear_base_radius,
                                                     0.5 * m_gearPinGeom->tooth_width, shape_offset,
                                                     Q_from_AngAxis(CH_C_PI_2, VECT_X));

            break;
        }
        case CollisionType::Mesh: {
            // use a triangle mesh

            geometry::ChTriangleMeshSoup temp_trianglemesh;

            // TODO: fill the triangleMesh here with some track shoe geometry

            // is there an offset??
            double shoelength = 0.2;
            ChVector<> mesh_displacement(shoelength * 0.5, 0, 0);  // since mesh origin is not in body center of mass
            m_gear->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

            break;
        }
        case CollisionType::ConvexHull: {
            // use convex hulls, loaded from file
            ChStreamInAsciiFile chull_file(GetChronoDataFile("drive_gear.chulls").c_str());
            // transform the collision geometry as needed
            double mangle = 45.0;  // guess
            ChQuaternion<> rot;
            rot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
            ChMatrix33<> rot_offset(rot);
            ChVector<> disp_offset(0, 0, 0);  // no displacement offset
            m_gear->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
            break;
        }
        case CollisionType::CallbackFunction: {
            // a set of boxes to represent the top-most flat face of the gear tooth
            // as the gear should be oriented initially with the tooth base directly
            // above the COG, each tooth box is rotated from the initial half rotation angle
            double init_rot =
                CH_C_PI /
                m_gearPinGeom->num_teeth;  // std::atan(0.07334/0.24929); // from sprocket geometry blender file
            for (size_t b_idx = 0; b_idx < m_gearPinGeom->num_teeth; b_idx++) {
                // this is the angle from the vertical (y-axis local y c-sys).
                double rot_ang = init_rot + 2.0 * init_rot * b_idx;
                // distance center of tooth is from the gear spin axis
                double len_from_rotaxis =
                    ChVector<>(m_gearPinGeom->tooth_mid_bar.x, m_gearPinGeom->tooth_mid_bar.y, 0).Length();
                // shift the box vertically, midpoint of base should be at center of gear
                // if the rotation is relative to sprocket COG, then this will end up in the right place
                ChVector<> box_center(0.5 * len_from_rotaxis * std::sin(rot_ang),  // 0
                                      0.5 * len_from_rotaxis * std::cos(rot_ang),  //  0.5*len_from_rotaxis
                                      m_gearPinGeom->tooth_mid_bar.z);

                // z-axis is out of the page, to rotate clockwise negate the rotation angle.
                ChMatrix33<> box_rot_mat(Q_from_AngAxis(-rot_ang, VECT_Z));

                m_gear->GetCollisionModel()->AddBox(
                    0.5 * m_gearPinGeom->tooth_len, 0.5 * len_from_rotaxis, 0.5 * m_gearPinGeom->tooth_width,
                    box_center,
                    box_rot_mat);  // does this rotation occur about gear c-sys or center of box ????

                // the gear teeth are symmetric about XY plane
                box_center.z *= -1;
                m_gear->GetCollisionModel()->AddBox(
                    0.5 * m_gearPinGeom->tooth_len, 0.5 * len_from_rotaxis, 0.5 * m_gearPinGeom->tooth_width,
                    box_center,
                    box_rot_mat);  // does this rotation occur about gear c-sys or center of box ????
            }

            // add the gear and shoes in this chain to the collision callback class
            vehicle->AddGearPinCollisionCallback(shoes, m_gear, m_gearPinGeom);

            break;
        }
        default:
            // no collision geometry
            GetLog() << "not recognized CollisionType: " << (int)m_collide << " for drive gear \n";
            m_gear->SetCollide(false);
            return;
    }  // end switch

    // set collision family, gear is a rolling element like the wheels
    m_gear->GetCollisionModel()->SetFamily((int)CollisionFam::Gear);

    // only collide w/ shoes on the same side of the vehicle
    if (side == RIGHTSIDE) {
        m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeLeft);
    } else {
        m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeRight);
    }

    // don't collide with other rolling elements
    m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Ground);
    m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Gear);
    m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Wheel);
    m_gear->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Hull);

    m_gear->GetCollisionModel()->BuildModel();
}

// helper functions
double DriveGearMotion::clamp(double val, double min_val, double max_val) {
    if (val <= min_val)
        return min_val;
    if (val >= max_val)
        return max_val;
    return val;
}

void DriveGearMotion::Write_header(const std::string& filename, DebugType type) {
    if (type & DBG_BODY) {
        m_filename_DBG_BODY = filename;
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str());
        // headers
        ofile << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz\n";
    }

    if (type & DBG_CONTACTS) {
        // report on some specific collision info in a separate file
        // filename
        m_filename_DBG_CONTACTS = filename;
        ChStreamOutAsciiFile ofile(m_filename_DBG_CONTACTS.c_str());
        // headers
        ofile << "time,Ncontacts,FnMax,FnAvg,FnVar,FnSig,FtMax,FtAvg,FtVar,FtSig\n";
    }
    if (type & DBG_CONSTRAINTS) {
        m_filename_DBG_CV = filename;
        ChStreamOutAsciiFile ofile(m_filename_DBG_CV.c_str());
        // headers
        ofile << "time,x,y,z,rx,ry,rz\n";
    }
}

void DriveGearMotion::Write_data(const double t, ChSystem* system, DebugType type) {
    if (type & DBG_BODY) {
        std::stringstream ss_g;
        ChSharedPtr<ChBody> gb = GetBody();
        // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz
        ss_g << t << "," << gb->GetPos() << "," << gb->GetPos_dt() << "," << gb->GetWvel_loc() << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str(), std::ios::app);
        ofile << ss_g.str().c_str();
    }
    if (type & DBG_CONTACTS) {
        // find what's in contact with the gear by processing all collisions with a special callback function
        ChVector<> Fn_info = ChVector<>();
        ChVector<> Ft_info = ChVector<>();
        // info is: (max, avg., variance)
        int num_gear_contacts = reportGearContact(GetBody(), system, Fn_info, Ft_info);

        std::stringstream ss_gc;
        // time,Ncontacts,FnMax,FnAvg,FnVar,FtMax,FtAvg,FtVar
        ss_gc << t << "," << num_gear_contacts << "," << Fn_info << "," << std::sqrt(Fn_info.z) << "," << Ft_info << ","
              << std::sqrt(Ft_info.z) << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_CONTACTS.c_str(), std::ios::app);
        ofile << ss_gc.str().c_str();
    }
    if (type & DBG_CONSTRAINTS) {
        std::stringstream ss;
        ss << t;
        ChMatrix<>* C = m_revolute->GetC();
        for (int row = 0; row < C->GetRows(); row++) {
            ss << "," << C->GetElement(row, 0);
        }
        ss << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_CV.c_str(), std::ios::app);
        ofile << ss.str().c_str();
    }
}

// get some data about the gear and its normal and friction contact forces each timestep.
// info = (max, avg, stdev = sigma)
// returns: number of contacts for the gear body.
int DriveGearMotion::reportGearContact(const ChSharedPtr<ChBody> which_gear,
                                       ChSystem* system,
                                       ChVector<>& Fn_info,
                                       ChVector<>& Ft_info) {
    // setup the reporter, init any variables
    _gear_report_contact reporter(which_gear);

    // pass the reporter callback to the system
    system->GetContactContainer()->ReportAllContacts(&reporter);

    // set any data here from the reporter, and return # of bodies in contact with the gear
    double Fn_avg = (reporter.m_num_gear_contacts > 0) ? reporter.m_Fn_sum / reporter.m_num_gear_contacts : 0;
    Fn_info = ChVector<>(reporter.m_Fn_max, Fn_avg, std::sqrt(reporter.m_Fn_var));

    double Ft_avg = (reporter.m_num_gear_contacts > 0) ? reporter.m_Ft_sum / reporter.m_num_gear_contacts : 0;
    Ft_info = ChVector<>(reporter.m_Ft_max, Ft_avg, std::sqrt(reporter.m_Ft_var));

    return reporter.m_num_gear_contacts;
}

}  // end namespace chrono
