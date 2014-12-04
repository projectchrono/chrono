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
#include "rapidjson/filereadstream.h"

#include "TrackChain.h"

using namespace rapidjson;

namespace chrono {

// JSON utility functions
static ChVector<> loadVector(const Value& a)
{
  assert(a.IsArray());
  assert(a.Size() == 3);

  return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}


TrackChain::TrackChain(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

TrackChain::TrackChain(const rapidjson::Document& d)
{
  Create(d);
}

TrackChain::Create(const rapidjson::Document& d)
{
  // check that required categories are present
  assert(d.HasMember("Type"));
  assert(d.HasMember("Name"));
  assert(d.HasMember("Pin Distance"));
  assert(d.HasMember("Collision"));
  assert(d.HasMember("Visualization"));
  
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
}

void TrackChain::Initialize(std::vector<ChBody>& control_bodies,
							std::vector<double> clearance,
							const ChVector<>& start_loc)
{
  // Express the steering reference frame in the absolute coordinate system.
  ChFrame<> idler_to_abs(location, rotation);
  idler_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

  
  
}

void TrackChain::AddVisualizationIdler()
{

}

} // end namespace chrono
