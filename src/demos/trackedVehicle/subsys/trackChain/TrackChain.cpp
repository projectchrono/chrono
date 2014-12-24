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

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"


namespace chrono {


// static variables
const std::string TrackChain::m_collision_filename = "wheel_L_POV_geom";
const std::string TrackChain::m_visual_filename = utils::GetModelDataFile("hmmwv/wheel_L.obj");

const double TrackChain::m_mass = 45.0;
const ChVector<> TrackChain::m_inertia(2.0,3.0,4.0);
const double TrackChain::m_shoe_width = 0.4;
const double TrackChain::m_shoe_height = 0.2;
const double TrackChain::m_pin_dist = 0.3;		// linear distance between a shoe's two pin joint center
const double TrackChain::m_pin_radius = 0.05;

TrackChain::TrackChain(const std::string& name, VisualizationType vis, CollisionType collide)
{
  m_shoes.clear();
  m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));

  // Attach visualization to the base track shoe (e.g., m_shoes[0] )
   // 0 = none, 1 = primitive, 2 = mesh
  m_visType = 1;      // HARDCODED visType
  switch (m_visType) {
  case 1:
  {
    // primitive box, also used for collision.
    // shoes will be added to the same collision family so self-collision can be toggled
    ChSharedPtr<ChBoxShape> box(new ChBoxShape);
    box->GetBoxGeometry().SetLengths( 0.5 * ChVector<>( m_pin_dist-m_pin_radius, m_shoe_height, m_shoe_width) );
    m_shoes[0]->AddAsset(box);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    m_shoes[0]->AddAsset(tex);

    break;
  }
  case 2:
  {
    // mesh for visualization only.
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(get_visual_filename(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(get_visual_filename());
    m_shoes[0]->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_shoes[0]->AddAsset(mcolor);

    break;
  }
  }
}

int TrackChain::Create()
{
  /*
  // load data
  m_PinDist = d["Pin Distance"].GetDouble();
  m_collision_filename = d["Collision"]["Filename"].GetString();
  m_visual_filename = d["Visualization"]["Filename"].GetString();
  
  // load visualization mesh
  ChTriangleMeshConnected triangles;
  triangles.LoadWavefrontMesh(utils::GetModelDataFile(m_visual_filename), false, false);
  m_geom_visual = ChSharedPtr<ChTriangleMeshShape>(new ChTriangleMeshShape);
  m_geom_visual->SetMesh(triangles);
  m_geom_visual->SetName("shoe");

  */

  return 0;
}

void TrackChain::Initialize(std::vector<ChBody>& control_bodies,
							std::vector<double> clearance,
							const ChVector<>& start_loc)
{
  // Express the steering reference frame in the absolute coordinate system.
  // ChFrame<> idler_to_abs(location, rotation);
  // idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  
  
}

void TrackChain::AddVisualizationIdler()
{

}

} // end namespace chrono
