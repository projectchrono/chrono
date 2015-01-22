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
// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"


namespace chrono {

// Static variables
const double IdlerSimple::m_mass = 429.6;
const ChVector<> IdlerSimple::m_inertia = ChVector<>(14.7, 12.55, 12.55);

const std::string IdlerSimple::m_meshName = "idler_mesh";
const std::string IdlerSimple::m_meshFile = utils::GetModelDataFile("data/idlerMesh.obj");

// guessing at these values
const double IdlerSimple::m_radius = 0.255;
const double IdlerSimple::m_width = 0.166;
const double IdlerSimple::m_widthGap = 0.092; 
const double IdlerSimple::m_springK = 100000;
const double IdlerSimple::m_springC = 1000;
const double IdlerSimple::m_springRestLength = 1.0;


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
                         VisualizationType vis,
                         CollisionType collide)
  : m_vis(vis), m_collide(collide)
//  , m_shockCB(NULL), m_springCB(NULL)
{
  // create the body, set the basic info
  m_idler = ChSharedPtr<ChBody>(new ChBody);
  m_idler->SetNameString(name + "_body");
  m_idler->SetMass(m_mass);
  m_idler->SetInertiaXX(m_inertia);

  // create the idler joint
  m_idler_joint = ChSharedPtr<ChLinkLockRevolutePrismatic>(new ChLinkLockRevolutePrismatic);
  m_idler_joint->SetNameString(name + "_idler_joint");

  // create the tensioning linear spring-shock
  m_shock = ChSharedPtr<ChLinkSpring>(new ChLinkSpring);
  m_shock->SetNameString(name + "_shock");
  m_shock->Set_SpringK(m_springK);
  m_shock->Set_SpringR(m_springC);
  m_shock->Set_SpringRestLength(m_springRestLength);

  AddVisualization();
 
}

IdlerSimple::~IdlerSimple()
{
  // delete m_springCB;
  // delete m_shockCB;
}

void IdlerSimple::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                             const ChCoordsys<>& local_Csys)
{
  // add collision geometry
  AddCollisionGeometry();

  // Express the steering reference frame in the absolute coordinate system.
  ChFrame<> idler_to_abs(local_Csys);
  idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  // transform the idler body, add to system
  m_idler->SetPos(idler_to_abs.GetPos());
  m_idler->SetRot(idler_to_abs.GetRot());
  chassis->GetSystem()->Add(m_idler);
  
  // init joint, add to system
  // body 1 should rotate about z-axis, translate about x-axis of body2
  // TODO: (check) idler joint translates, rotates in correct direction.
  // NOTE: I created the idler to translate x-dir, rotate about z-dir, according
  //      to how the chassis is rotated by default.
  m_idler_joint->Initialize(m_idler, chassis, m_idler->GetCoord() );
  chassis->GetSystem()->AddLink(m_idler_joint);

  // init shock, add to system
  // put the second marker some length in front of marker1, based on desired preload
  double preLoad = 10000; // [N]
  // chassis spring attachment point is towards the center of the vehicle
  ChVector<> pos_chassis_abs = local_Csys.pos;
  if(local_Csys.pos.x < 0 ) 
    pos_chassis_abs.x += m_springRestLength - (preLoad / m_springK);
  else
    pos_chassis_abs.x -= m_springRestLength - (preLoad / m_springK);

  // transform 2nd attachment point to abs coords
  pos_chassis_abs = chassis->GetCoord().TransformPointLocalToParent(pos_chassis_abs);

  // init. points based on desired preload and free lengths
  m_shock->Initialize(m_idler, chassis, false, m_idler->GetPos(), pos_chassis_abs );
  // setting rest length should yield desired preload at time = 0
  m_shock->Set_SpringRestLength(m_springRestLength);

  chassis->GetSystem()->AddLink(m_shock);
}

void IdlerSimple::AddVisualization()
{
  // add visualization asset
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    // define the shape with two concentric cyclinders, with a gap.
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, m_width/2.0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, 0, m_widthGap/2.0);
    cyl->GetCylinderGeometry().rad = m_radius;
    m_idler->AddAsset(cyl);

    // second cylinder is a mirror of the first
    cyl->GetCylinderGeometry().p1.z = -m_width/2.0;
    cyl->GetCylinderGeometry().p2.z = -m_widthGap/2.0;
    m_idler->AddAsset(cyl);

    // add a color asset
    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.5f, 0.1f, 0.4f));
    m_idler->AddAsset(mcolor);

    break;
  }
   case VisualizationType::MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_idler->AddAsset(trimesh_shape);

    // add a color asset
    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.5f, 0.1f, 0.4f));
    m_idler->AddAsset(mcolor);

    break;
  }
  } // end switch
}

void IdlerSimple::AddCollisionGeometry()
{
  // add collision geometrey to the chassis, if enabled
  m_idler->SetCollide(true);
  m_idler->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::NONE:
    {
      m_idler->SetCollide(false);
    }
  case CollisionType::PRIMITIVES:
  {
    double half_cyl_width =  (m_width - m_widthGap)/2.0;
    ChVector<> shape_offset =  ChVector<>(0, 0, half_cyl_width + m_widthGap/2.0);
    // use two cylinders.
    m_idler->GetCollisionModel()->AddCylinder(m_radius, m_radius, half_cyl_width,
      shape_offset,Q_from_AngAxis(CH_C_PI_2,VECT_X));

    // mirror first cylinder about the x-y plane
    shape_offset.z *= -1;
    m_idler->GetCollisionModel()->AddCylinder(m_radius, m_radius, half_cyl_width,
      shape_offset,Q_from_AngAxis(CH_C_PI_2,VECT_X));

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		

    // TODO: fill the triangleMesh here with some track shoe geometry

    
		m_idler->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_idler->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_idler->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_idler->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("idler.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_idler->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    break;
  }
  } // end switch

  // setup collision family
  m_idler->GetCollisionModel()->SetFamily( (int)CollisionFam::GEARS );
  // don't collide with other roling elements or the ground
  m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::HULL );
  m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::GEARS );
  m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::WHEELS );
  m_idler->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::GROUND );

  m_idler->GetCollisionModel()->BuildModel();
}

} // end namespace chrono
