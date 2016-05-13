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
// A Simple Idler system that keeps track chain tension by pre-loading a
//  spring/damper elemnt
//
// =============================================================================

#include <cstdio>

#include "IdlerSimple.h"

#include "assets/ChAsset.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChColorAsset.h"
#include "assets/ChTexture.h"

#include "geometry/ChTriangleMeshSoup.h"

#include "utils/ChUtilsInputOutput.h"

#include "subsys/ChVehicleModelData.h"

namespace chrono {

// guessing at these values
const double IdlerSimple::m_radius = 0.255;
const double IdlerSimple::m_width = 0.166 * 2.4;
const double IdlerSimple::m_widthGap = .092 * 1.5;  // 0.092;

/*
// -----------------------------------------------------------------------------
// Default shock and spring functors (used for linear elements)
// -----------------------------------------------------------------------------
class LinearSpringForce : public ChSpringForceCallback
{
public:
  LinearSpringForce(double k) : m_k(k) {}

  virtual double operator()(double time,         // current time
                            double rest_length,  // undeformed length
                            double length,       // current length
                            double vel)          // current velocity (positive when extending)
  {
    return -m_k * (length - rest_length);
  }

private:
  double  m_k;
};

class LinearShockForce : public ChSpringForceCallback
{
public:
  LinearShockForce(double c) : m_c(c) {}

  virtual double operator()(double time,         // current time
                            double rest_length,  // undeformed length
                            double length,       // current length
                            double vel)          // current velocity (positive when extending)
  {
    return -m_c * vel;
  }

private:
  double  m_c;
};
*/

IdlerSimple::IdlerSimple(const std::string& name,
                         VisualizationType::Enum vis,
                         CollisionType::Enum collide,
                         size_t chainSys_idx,
                         double mass,
                         const ChVector<>& Ixx,
                         double tensionerK,
                         double tensionerC,
                         double springFreeLen,
                         double mu)
    : m_vis(vis),
      m_collide(collide),
      m_chainSys_idx(chainSys_idx),
      m_mass(mass),
      m_inertia(Ixx),
      m_tensionerK(tensionerK),
      m_tensionerC(tensionerC),
      m_meshFile(vehicle::GetDataFile("M113/Idler_XforwardYup.obj")),
      m_meshName("idler_mesh"),
      m_springRestLength(springFreeLen),
      m_mu(mu)

//  , m_shockCB(NULL), m_springCB(NULL)
{
    // create the body, set the basic info
    m_idler = ChSharedPtr<ChBody>(new ChBody);
    m_idler->SetNameString(name + "_body");

    // use the input values rather than statically defined
    m_idler->SetMass(m_mass);
    m_idler->SetInertiaXX(m_inertia);

    // create the idler joint
    m_idler_joint = ChSharedPtr<ChLinkLockRevolutePrismatic>(new ChLinkLockRevolutePrismatic);
    m_idler_joint->SetNameString(name + "_idler_joint");

    // create the tensioning linear spring-shock
    m_shock = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
    m_shock->SetNameString(name + "_shock");
    m_shock->Set_SpringK(m_tensionerK);
    m_shock->Set_SpringR(m_tensionerC);
    m_shock->Set_SpringRestLength(m_springRestLength);

    AddVisualization(m_chainSys_idx);
}

IdlerSimple::~IdlerSimple() {
    // delete m_springCB;
    // delete m_shockCB;
}

void IdlerSimple::Initialize(ChSharedPtr<ChBody> chassis,
                             const ChFrame<>& chassis_REF,
                             const ChCoordsys<>& local_Csys,
                             double preLoad) {
    // add collision geometry for the idler wheel
    (local_Csys.pos.z < 0) ? AddCollisionGeometry(LEFTSIDE, m_mu, 0.9*m_mu) : AddCollisionGeometry(RIGHTSIDE, m_mu, 0.9*m_mu);

    // Express the reference frame in the absolute coordinate system.
    ChFrame<> idler_to_abs(local_Csys);
    idler_to_abs.ConcatenatePreTransformation(chassis_REF);

    // transform the idler body, add to system
    m_idler->SetPos(idler_to_abs.GetPos());
    m_idler->SetRot(idler_to_abs.GetRot());
    chassis->GetSystem()->Add(m_idler);

    // init joint, add to system
    // body 1 should rotate about z-axis, translate about x-axis of body2
    // TODO: (check) idler joint translates, rotates in correct direction.
    // NOTE: I created the idler to translate x-dir, rotate about z-dir, according
    //      to how the chassis is rotated by default.
    m_idler_joint->Initialize(m_idler, chassis, ChCoordsys<>(idler_to_abs.GetPos(), idler_to_abs.GetRot()));
    chassis->GetSystem()->AddLink(m_idler_joint);

    // init shock, add to system
    // put the second marker some length in front of marker1, based on desired preload
    double init_spring_len = m_springRestLength - preLoad / m_tensionerK;
    double min_init_spring_len = 0.1;
    if (init_spring_len < min_init_spring_len) {
        // must increase restLength. Set init_spring_len to a constant 10 cm
        m_springRestLength += abs(init_spring_len) + 0.1;
        init_spring_len = min_init_spring_len;
    }
    ChVector<> pos_on_chassis_abs = idler_to_abs.GetPos() - idler_to_abs.GetRot().GetXaxis() * (init_spring_len);

    // init. points based on desired preload and free lengths
    m_shock->Initialize(m_idler, chassis, false, m_idler->GetPos(), pos_on_chassis_abs);
    // setting rest length should yield desired preload at time = 0
    m_shock->Set_SpringRestLength(m_springRestLength);

    chassis->GetSystem()->AddLink(m_shock);
}

void IdlerSimple::AddVisualization(size_t chain_idx, bool custom_texture, const std::string& tex_name) {
    if (m_idler->GetAssets().size() > 0)
        m_idler->GetAssets().clear();

    // add visualization asset
    switch (m_vis) {
        case VisualizationType::Primitives: {
            ChSharedPtr<ChCylinderShape> cylA(new ChCylinderShape);
            // define the shape with two concentric cyclinders, with a gap.
            cylA->GetCylinderGeometry().p1 = ChVector<>(0, 0, m_width / 2.0);
            cylA->GetCylinderGeometry().p2 = ChVector<>(0, 0, m_widthGap / 2.0);
            cylA->GetCylinderGeometry().rad = m_radius;
            m_idler->AddAsset(cylA);

            // second cylinder is a mirror of the first
            ChSharedPtr<ChCylinderShape> cylB(new ChCylinderShape(*cylA.get_ptr()));
            cylB->GetCylinderGeometry().p1.z = -m_width / 2.0;
            cylB->GetCylinderGeometry().p2.z = -m_widthGap / 2.0;
            m_idler->AddAsset(cylB);

            // add a texture asset.
            ChSharedPtr<ChTexture> cyl_tex(new ChTexture);
            if (custom_texture)
                cyl_tex->SetTextureFilename(GetChronoDataFile(tex_name));
            else {
                cyl_tex->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
            }
            m_idler->AddAsset(cyl_tex);

            break;
        }
        case VisualizationType::Mesh: {
            geometry::ChTriangleMeshConnected trimesh;
            trimesh.LoadWavefrontMesh(getMeshFile(), true, false);

            ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetName(getMeshName());
            m_idler->AddAsset(trimesh_shape);

            ChSharedPtr<ChTexture> cyl_tex(new ChTexture);
            if (custom_texture)
                cyl_tex->SetTextureFilename(GetChronoDataFile(tex_name));
            else {
                cyl_tex->SetTextureFilename(GetChronoDataFile("greenwhite.png"));
            }
            m_idler->AddAsset(cyl_tex);

            break;
        }
        default: { GetLog() << "Didn't recognize VisualizationType for IdlerSimple \n"; }
    }  // end switch
}

void IdlerSimple::AddCollisionGeometry(VehicleSide side, double mu, double mu_sliding, double mu_roll, double mu_spin) {
    // add collision geometrey to the chassis, if enabled
    if (m_collide == CollisionType::None) {
        m_idler->SetCollide(false);
        GetLog() << " !!! Idler " << m_idler->GetName() << " collision deactivated !!! \n\n";
        return;
    }

    m_idler->SetCollide(true);
    m_idler->GetCollisionModel()->ClearModel();

    m_idler->GetCollisionModel()->SetSafeMargin(0.002);  // inward safe margin
    m_idler->GetCollisionModel()->SetEnvelope(0.005);    // distance of the outward "collision envelope"

    // set the collision material
    m_idler->GetMaterialSurface()->SetSfriction(mu);
    m_idler->GetMaterialSurface()->SetKfriction(mu_sliding);
    m_idler->GetMaterialSurface()->SetRollingFriction(mu_roll);
    m_idler->GetMaterialSurface()->SetSpinningFriction(mu_spin);

    switch (m_collide) {
        case CollisionType::Primitives: {
            double cyl_width = 0.5 * (m_width - m_widthGap);
            ChVector<> shape_offset = ChVector<>(0, 0, 0.5 * (cyl_width + m_widthGap));
            // use two cylinders.
            m_idler->GetCollisionModel()->AddCylinder(m_radius, m_radius, 0.5 * cyl_width, shape_offset,
                                                      Q_from_AngAxis(CH_C_PI_2, VECT_X));

            // mirror first cylinder about the x-y plane
            shape_offset.z *= -1;
            m_idler->GetCollisionModel()->AddCylinder(m_radius, m_radius, 0.5 * cyl_width, shape_offset,
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
            m_idler->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

            break;
        }
        case CollisionType::ConvexHull: {
            // use convex hulls, loaded from file
            ChStreamInAsciiFile chull_file(GetChronoDataFile("idler.chulls").c_str());
            // transform the collision geometry as needed
            double mangle = 45.0;  // guess
            ChQuaternion<> rot;
            rot.Q_from_AngAxis(mangle * (CH_C_PI / 180.), VECT_X);
            ChMatrix33<> rot_offset(rot);
            ChVector<> disp_offset(0, 0, 0);  // no displacement offset
            m_idler->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
            break;
        }
        default:
            // no collision geometry
            GetLog() << "not recognized CollisionType: " << (int)m_collide << " for idler wheel \n";
            m_idler->SetCollide(false);
            return;
    }  // end switch

    // setup collision family, idler is a rolling element
    m_idler->GetCollisionModel()->SetFamily((int)CollisionFam::Wheel);

    // only collide w/ shoes on the same side of the vehicle
    if (side == RIGHTSIDE) {
        m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeLeft);
    } else {
        m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::ShoeRight);
    }

    // don't collide with the other wheels, nor ground
    m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Wheel);
    m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Hull);
    m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Gear);
    m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily((int)CollisionFam::Ground);

    m_idler->GetCollisionModel()->BuildModel();
}

double IdlerSimple::Get_SpringReact_Deform() const {
    // force from spring
    double deform = m_shock->Get_SpringDeform();
    double deform_dt = m_shock->Get_SpringVelocity();
    double spring_react_K = -(m_shock->Get_SpringK() * (m_shock->Get_mod_k_d())->Get_y(deform) *
                              (m_shock->Get_mod_k_speed())->Get_y(deform_dt)) *
                            deform;

    return spring_react_K;
}

double IdlerSimple::Get_SpringReact_Deform_dt() const {
    // force from damping
    double deform = m_shock->Get_SpringDeform();
    double deform_dt = m_shock->Get_SpringVelocity();
    double spr_react_C = -(m_shock->Get_SpringR() * (m_shock->Get_mod_r_d())->Get_y(deform) *
                           (m_shock->Get_mod_r_speed())->Get_y(deform_dt)) *
                         deform_dt;

    return spr_react_C;
}

// ---------------------------------
// write to file functions
// --------------------------------
void IdlerSimple::Write_header(const std::string& filename, DebugType type) {
    if (type & DBG_BODY) {
        m_filename_DBG_BODY = filename;
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str());
        // headers
        ofile << "time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c\n";
    }
    if (type & DBG_CONTACTS) {
        // todo
    }
    if (type & DBG_CONSTRAINTS) {
        m_filename_DBG_CV = filename;
        ChStreamOutAsciiFile ofile(m_filename_DBG_CV.c_str());
        // headers
        ofile << "time,y,z,rx,ry\n";
    }
}

void IdlerSimple::Write_data(const double t, DebugType type) {
    if (type & DBG_BODY) {
        std::stringstream ss_id;
        ChSharedPtr<ChBody> ib = GetBody();
        // time,x,y,z,Vx,Vy,Vz,Wx,Wy,Wz,F_tensioner,F_k,F_c
        ss_id << t << "," << ib->GetPos() << "," << ib->GetPos_dt() << "," << ib->GetWvel_loc() << ","
              << GetSpringForce() << "," << Get_SpringReact_Deform() << "," << Get_SpringReact_Deform_dt() << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_BODY.c_str(), std::ios::app);
        ofile << ss_id.str().c_str();
    }
    if (type & DBG_CONTACTS) {
        // todo
    }
    if (type & DBG_CONSTRAINTS) {
        std::stringstream ss;
        ss << t;
        // idler joint will have y and z rxn forces, x and y rxn torques
        ChMatrix<>* C = m_idler_joint->GetC();
        for (int row = 0; row < C->GetRows(); row++) {
            ss << "," << C->GetElement(row, 0);
        }
        ss << "\n";
        ChStreamOutAsciiFile ofile(m_filename_DBG_CV.c_str(), std::ios::app);
        ofile << ss.str().c_str();
    }
}

}  // end namespace chrono
