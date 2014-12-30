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
// Generates a track chain aropund the track system
//
// =============================================================================

#include <cstdio>

#include "TrackChain.h"

#include "physics/ChBody.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
// collision mesh
#include "geometry/ChCTriangleMeshSoup.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"


namespace chrono {


// static variables
const std::string TrackChain::m_collisionFile = utils::GetModelDataFile("track_data/M113/shoe_collision.obj");
const std::string TrackChain::m_meshName = "M113 shoe"; 
const std::string TrackChain::m_meshFile = utils::GetModelDataFile("track_data/M113/shoe_view.obj");

const double TrackChain::m_mass = 18.02;
const ChVector<> TrackChain::m_inertia(0.04, 0.22, 0.25);
const double TrackChain::m_shoe_width = 0.4;
const double TrackChain::m_shoe_height = 0.2;
const double TrackChain::m_pin_dist = 0.3;		// linear distance between a shoe's two pin joint center
const double TrackChain::m_pin_radius = 0.05;

TrackChain::TrackChain(const std::string& name, 
                       VisualizationType vis, 
                       CollisionType collide)
                       : m_vis(vis), m_collide(collide), m_numShoes(0)
{
  // clear vector holding list of body handles
  m_shoes.clear();
  // add first track shoe body
  m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));
  m_shoes[0]->SetNameString("shoe 0, "+name);
  m_shoes[0]->SetMass(m_mass);
  m_shoes[0]->SetInertiaXX(m_inertia);
  m_numShoes++;

  // Attach visualization to the base track shoe
  AddVisualization(0);


}


void TrackChain::Initialize(ChSharedPtr<ChBodyAuxRef> chassis,
                            const std::vector<ChVector<>>& control_points,
                            const std::vector<double>& clearance,
                            const ChVector<>& start_loc)
{
  // add collision geometry to the first track shoe
  AddCollisionGeometry(0);
  m_numShoes = 1;

  // ChFrame<> idler_to_abs(location, rotation);
  // idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
  

  // hard part: "wrap" the track chain around the trackSystem, e.g., drive-gear,
  // idler, road-wheels. First and last shoes are allowed to be in any orientation,
  // as long as the final pin joint connects correctly.
  CreateChain(control_points, clearance, start_loc );
}

void TrackChain::AddVisualization(size_t track_idx)
{
  assert(track_idx < m_numShoes);
  // Attach visualization asset
  switch (m_vis) {
  case VisualizationType::PRIMITIVES:
  {
    // primitive box, also used for collision.
    // shoes will be added to the same collision family so self-collision can be toggled
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().SetLengths( 0.5 * ChVector<>( m_pin_dist-2*m_pin_radius, m_shoe_height, m_shoe_width) );
    m_shoes[track_idx]->AddAsset(box);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    m_shoes[track_idx]->AddAsset(tex);

    break;
  }
  case VisualizationType::MESH:
  {
    // mesh for visualization only.
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_shoes[track_idx]->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_shoes[track_idx]->AddAsset(mcolor);

    break;
  }
  }
}


void TrackChain::AddCollisionGeometry(size_t track_idx)
{
  assert(track_idx < m_numShoes);
   // add collision geometrey to the chassis, if enabled
  m_shoes[track_idx]->SetCollide(true);
  m_shoes[track_idx]->GetCollisionModel()->ClearModel();

  switch (m_collide) {
  case CollisionType::PRIMITIVES:
  {
    // use a simple box
    m_shoes[track_idx]->GetCollisionModel()->AddBox(m_pin_dist-2*m_pin_radius, m_shoe_height, m_shoe_width);

    break;
  }
  case CollisionType::MESH:
  {
    // use a triangle mesh
   
		geometry::ChTriangleMeshSoup temp_trianglemesh; 
		
    // TODO: fill the triangleMesh here with some track shoe geometry

		m_shoes[track_idx]->GetCollisionModel()->SetSafeMargin(0.004);	// inward safe margin
		m_shoes[track_idx]->GetCollisionModel()->SetEnvelope(0.010);		// distance of the outward "collision envelope"
		m_shoes[track_idx]->GetCollisionModel()->ClearModel();

    // is there an offset??
    double shoelength = 0.2;
    ChVector<> mesh_displacement(shoelength*0.5,0,0);  // since mesh origin is not in body center of mass
    m_shoes[track_idx]->GetCollisionModel()->AddTriangleMesh(temp_trianglemesh, false, false, mesh_displacement);

    break;
  }
  case CollisionType::CONVEXHULL:
  {
    // use convex hulls, loaded from file
    ChStreamInAsciiFile chull_file(GetChronoDataFile("track_data/M113/shoe_collision.chulls").c_str());
    // transform the collision geometry as needed
    double mangle = 45.0; // guess
    ChQuaternion<>rot;
    rot.Q_from_AngAxis(mangle*(CH_C_PI/180.),VECT_X);
    ChMatrix33<> rot_offset(rot);
    ChVector<> disp_offset(0,0,0);  // no displacement offset
    m_shoes[track_idx]->GetCollisionModel()->AddConvexHullsFromFile(chull_file, disp_offset, rot_offset);
    break;
  }
  } // end switch

  // set collision family
  m_shoes[track_idx]->GetCollisionModel()->SetFamily( (int)CollisionFam::SHOES);
  // don't collide with other shoes, but with everything else
  m_shoes[track_idx]->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily( (int)CollisionFam::SHOES );

  m_shoes[track_idx]->GetCollisionModel()->BuildModel();

}


void TrackChain::CreateChain(const std::vector<ChVector<>>& control_points,
                            const std::vector<double>& clearance,
                            const ChVector<>& start_loc)
{

}


ChSharedPtr<ChBody> TrackChain::GetShoeBody(size_t track_idx)
{
  assert( track_idx < m_numShoes);
  return (track_idx > m_numShoes-1) ? m_shoes[track_idx] : m_shoes[0] ;
}

} // end namespace chrono
