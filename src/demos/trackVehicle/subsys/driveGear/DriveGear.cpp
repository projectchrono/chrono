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
// The drive gear propels the tracked vehicle
//
// =============================================================================

#include "DriveGear.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "assets/ChAssetLevel.h"

#include "geometry/ChTriangleMeshSoup.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"

namespace chrono {

// static variables
const double DriveGear::m_shaft_inertia = 0.4;  // connects to driveline
// for primitive collision/visualization
// const double DriveGear::m_radius = 0.212; // to collision surface
// const double DriveGear::m_width = 0.34;
// const double DriveGear::m_widthGap = 0.189; // 0.189; // inner distance between cydliners

DriveGear::DriveGear(const std::string& name,
                     VisualizationType::Enum vis,
                     CollisionType::Enum collide,
                     size_t chainSys_idx,
                     double mass,
                     const ChVector<>& gear_Ixx)
    : m_vis(vis),
      m_collide(collide),
      m_meshFile(vehicle::GetDataFile("M113/Sprocket_XforwardYup.obj")),
      m_chainSys_idx(chainSys_idx),
      m_mass(mass),
      m_inertia(gear_Ixx),
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
    m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_revolute->SetNameString(name + "_revolute");

    // create the shaft components
    m_axle = ChSharedPtr<ChShaft>(new ChShaft);
    m_axle->SetNameString(name + "_axle");
    m_axle->SetInertia(m_shaft_inertia);
    // create the shaft to gear connection
    m_axle_to_gear = ChSharedPtr<ChShaftsBody>(new ChShaftsBody);
    m_axle_to_gear->SetNameString(name + "_axle_to_gear");

    // create the visuals
    AddVisualization();
}

DriveGear::~DriveGear() {
}

void DriveGear::Initialize(ChSharedPtr<ChBody> chassis,
                           const ChFrame<>& chassis_REF,
                           const ChCoordsys<>& local_Csys,
                           const std::vector<ChSharedPtr<ChBody> >& shoes,
                           ChTrackVehicle* vehicle) {
    assert(shoes.size() > 0);

    // add collision geometry
    VehicleSide chassis_side = RIGHTSIDE;
    if (local_Csys.pos.z < 0)
        chassis_side = LEFTSIDE;

    AddCollisionGeometry(shoes, vehicle, chassis_side);

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

    // initialize the axle shaft and connection to the drive gear body, add both to the system
    chassis->GetSystem()->Add(m_axle);
    m_axle_to_gear->Initialize(m_axle, m_gear, VECT_Z);
    chassis->GetSystem()->Add(m_axle_to_gear);
}

void DriveGear::AddVisualization() {
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

        default: { GetLog() << "Didn't recognize VisualizationType for DriveGear \n"; }
    }
}

void DriveGear::AddCollisionGeometry(const std::vector<ChSharedPtr<ChBody> >& shoes,
                                     ChTrackVehicle* vehicle,
                                     VehicleSide side,
                                     double mu,
                                     double mu_sliding,
                                     double mu_roll,
                                     double mu_spin) {
    // add collision geometrey, if enabled. Warn if disabled
    if (m_collide == CollisionType::None) {
        m_gear->SetCollide(false);
        GetLog() << " !!! DriveGear " << m_gear->GetName() << " collision deactivated !!! \n\n";
        return;
    }

    m_gear->SetCollide(true);
    m_gear->GetCollisionModel()->ClearModel();

    m_gear->GetCollisionModel()->SetSafeMargin(0.002);  // inward safe margin
    m_gear->GetCollisionModel()->SetEnvelope(0.005);    // distance of the outward "collision envelope"

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

            // NOTE: Custom callback doesn't work well when there is interpenetration,
            //    maintain the cylinder bodies as a gear seat base.
            // Only contributes when there is too much penetration between the gear seat and pin.
            // TODO: replace with boxes, so the contact normal will always be normal to the gear seat pos.,
            //  even when the pin slides off-center from the gear seat bottom.
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

void DriveGear::LogConstraintViolations() {
    // single revolute joint
    ChMatrix<>* C = m_revolute->GetC();
    GetLog() << " -- joint name: " << m_revolute->GetName();
    for (int row = 0; row < C->GetRows(); row++) {
        GetLog() << " " << C->GetElement(row, 0) << "  ";
    }
    GetLog() << "\n";
}

void DriveGear::SaveConstraintViolations(std::stringstream& ss) {
    // single revolute joint
    ChMatrix<>* C = m_revolute->GetC();
    for (int row = 0; row < C->GetRows(); row++) {
        ss << "," << C->GetElement(row, 0);
    }
    ss << "\n";
}

const std::string DriveGear::getFileHeader_ConstraintViolations(size_t idx) const {
    // gear is a revolute joint, z-rot DOF only
    std::stringstream ss;
    ss << "time,x,y,z,rx,ry\n";
    return ss.str();
}

}  // end namespace chrono
