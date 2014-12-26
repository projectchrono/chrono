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

TrackChain::TrackChain(const std::string& name, 
                       VisualizationType vis, 
                       CollisionType collide)
                       : m_vis(vis), m_collide(collide)
{
  m_shoes.clear();
  m_shoes.push_back(ChSharedPtr<ChBody>(new ChBody));

  // Attach visualization to the base track shoe (e.g., m_shoes[0] )
  AddVisualization(0);


}


void TrackChain::Initialize(std::vector<ChBody>& control_bodies,
							std::vector<double> clearance,
							const ChVector<>& start_loc)
{
  // add collision geometry to the first track shoe
  AddCollisionGeometry(0);
  m_numShoes = 1;

  // Express the steering reference frame in the absolute coordinate system.
  // ChFrame<> idler_to_abs(location, rotation);
  // idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
  

  // hard part: "wrap" the track chain around the trackSystem, e.g., drive-gear,
  // idler, road-wheels. First and last shoes are allowed to be in any orientation,
  // as long as the final pin joint connects correctly.
  CreateChain( );
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
    box->GetBoxGeometry().SetLengths( 0.5 * ChVector<>( m_pin_dist-m_pin_radius, m_shoe_height, m_shoe_width) );
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
    trimesh.LoadWavefrontMesh(get_visual_filename(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(get_visual_filename());
    m_shoes[track_idx]->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_shoes[track_idx]->AddAsset(mcolor);

    break;
  }
  }
}


void TrackChain::AddCollisionGeometry(size_t track_idx)
{

}


void TrackChain::CreateChain()
{

}

} // end namespace chrono
