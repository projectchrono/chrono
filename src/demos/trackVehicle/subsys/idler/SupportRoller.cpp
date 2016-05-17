// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// A support roller, with no suspension, connects to the hull via revolute constraint.
//
// =============================================================================

#include "SupportRoller.h"

#include "assets/ChCylinderShape.h"
// #include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
// #include "assets/ChColorAsset.h"
// collision mesh
// #include "geometry/ChTriangleMeshSoup.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"

namespace chrono {

// static variables
const double SupportRoller::m_radius = 0.2;
const double SupportRoller::m_width = 0.35;
const double SupportRoller::m_widthGap = 0.1;

SupportRoller::SupportRoller(const std::string& name,
                             VisualizationType::Enum vis,
                             CollisionType::Enum collide,
                             size_t chainSys_idx,
                             double mass,
                             const ChVector<>& Ixx)
    : m_vis(vis), m_collide(collide), m_chainSys_idx(chainSys_idx), m_mass(mass), m_inertia(Ixx) {
    // create the body, set the basic info
    m_roller = ChSharedPtr<ChBody>(new ChBody);
    m_roller->SetNameString(name);
    m_roller->SetMass(m_mass);
    m_roller->SetInertiaXX(m_inertia);

    // create the revolute joint
    m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_revolute->SetNameString(name + "_revolute");

    AddVisualization();
}

void SupportRoller::Initialize(ChSharedPtr<ChBody> chassis,
                               const ChFrame<>& chassis_REF,
                               const ChCoordsys<>& local_Csys) {
    // add collision geometry for the wheel
    (local_Csys.pos.z < 0) ? AddCollisionGeometry(LEFTSIDE) : AddCollisionGeometry();

    // get the local frame in the absolute ref. frame
    ChFrame<> frame_to_abs(local_Csys);
    frame_to_abs.ConcatenatePreTransformation(chassis_REF);

    // transform the drive gear body, add to system
    m_roller->SetPos(frame_to_abs.GetPos());
    m_roller->SetRot(frame_to_abs.GetRot());
    chassis->GetSystem()->Add(m_roller);

    // initialize the revolute joint, add to system
    m_revolute->Initialize(chassis, m_roller, ChCoordsys<>(frame_to_abs.GetPos(), frame_to_abs.GetRot()));
    chassis->GetSystem()->AddLink(m_revolute);
}

void SupportRoller::AddVisualization() {
    // Attach visualization asset
    switch (m_vis) {
        case VisualizationType::Primitives: {
            // define the gear as two concentric cylinders with a gap
            ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
            cyl->GetCylinderGeometry().rad = m_radius;
            cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, m_width / 2.0);
            cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, m_widthGap / 2.0);
            m_roller->AddAsset(cyl);

            // second cylinder is a mirror of the first, about x-y plane
            ChSharedPtr<ChCylinderShape> cylB(new ChCylinderShape(*cyl.get_ptr()));
            cylB->GetCylinderGeometry().p1.z *= -1;
            cylB->GetCylinderGeometry().p2.z *= -1;
            m_roller->AddAsset(cylB);

            ChSharedPtr<ChTexture> tex(new ChTexture);
            tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
            m_roller->AddAsset(tex);

            break;
        }
        /*
        case VisualizationType::Mesh:
        {
          geometry::ChTriangleMeshConnected trimesh;
          trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

          ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
          trimesh_shape->SetMesh(trimesh);
          trimesh_shape->SetName(getMeshName());
          m_roller->AddAsset(trimesh_shape);

          ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
          m_roller->AddAsset(mcolor);

          break;
        }
        */
        default: { GetLog() << "Didn't recognize VisualizationType for SupportRoller \n"; }
    }
}

void SupportRoller::AddCollisionGeometry(VehicleSide side,
                                         double mu,
                                         double mu_sliding,
                                         double mu_roll,
                                         double mu_spin) {
    // add collision geometrey, if enabled. Warn if disabled
    if (m_collide == CollisionType::None) {
        m_roller->SetCollide(false);
        GetLog() << " !!! SupportRoller " << m_roller->GetName() << " collision deactivated !!! \n\n";
        return;
    }

    m_roller->SetCollide(true);
    m_roller->GetCollisionModel()->ClearModel();

    m_roller->GetCollisionModel()->SetSafeMargin(0.001);  // inward safe margin
    m_roller->GetCollisionModel()->SetEnvelope(0.002);    // distance of the outward "collision envelope"

    // set the collision material
    m_roller->GetMaterialSurface()->SetSfriction(mu);
    m_roller->GetMaterialSurface()->SetKfriction(mu_sliding);
    m_roller->GetMaterialSurface()->SetRollingFriction(mu_roll);
    m_roller->GetMaterialSurface()->SetSpinningFriction(mu_spin);

    switch (m_collide) {
        case CollisionType::Primitives: {
            double cyl_width = 0.5 * (m_width - m_widthGap);
            ChVector<> shape_offset = ChVector<>(0, 0, 0.5 * (cyl_width + m_widthGap));
            // use two simple cylinders.
            m_roller->GetCollisionModel()->AddCylinder(m_radius, m_radius, 0.5 * cyl_width, shape_offset,
                                                       Q_from_AngAxis(CH_C_PI_2, VECT_X));

            // mirror first cylinder about the x-y plane
            shape_offset.z *= -1;
            m_roller->GetCollisionModel()->AddCylinder(m_radius, m_radius, 0.5 * cyl_width, shape_offset,
                                                       Q_from_AngAxis(CH_C_PI_2, VECT_X));

            break;
        }
        /*
        case CollisionType::Mesh:
        {
          // use a triangle mesh

          geometry::ChTriangleMeshSoup temp_trianglemesh;

          // TODO: fill the triangleMesh here with some track shoe geometry

          // is there an offset??
          double shoelength = 0.2;
          ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
          m_roller->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

          break;
        }
        */
        case CollisionType::ConvexHull: {
            // use convex hulls, loaded from file
            ChStreamInAsciiFile chull_file(GetChronoDataFile("drive_gear.chulls").c_str());
            // transform the collision geometry as needed
            double mangle = 45.0;  // guess
            ChQuaternion<> rot;
            rot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
            ChMatrix33<> rot_offset(rot);
            ChVector<> disp_offset(0, 0, 0);  // no displacement offset
            m_roller->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
            break;
        }
        default:
            // no collision geometry
            GetLog() << "not recognized CollisionType: " << (int)m_collide << " for drive gear \n";
            m_roller->SetCollide(false);
            return;
    }  // end switch

    // set collision family, gear is a rolling element like the wheels
    m_roller->GetCollisionModel()->SetFamily((int)CollisionFam::Wheel);

    // only collide w/ shoes on the same side of the vehicle
    if (side == RIGHTSIDE) {
        m_roller->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeLeft);
    } else {
        m_roller->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeRight);
    }
    // don't collide with other rolling elements
    m_roller->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Ground);
    m_roller->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Wheel);
    m_roller->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Hull);

    m_roller->GetCollisionModel()->BuildModel();
}

}  // end namespace chrono
