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

#include "subsys/trackChain/TrackChain.h"
#include "physics/ChBody.h"

namespace chrono {

TrackChain::TrackChain(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
 

  fclose(fp);

  Create();
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
