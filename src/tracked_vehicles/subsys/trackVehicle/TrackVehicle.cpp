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
// Tracked vehicle model built from subsystems specified w/ JSON input data file
//
// =============================================================================

#include <cstdio>

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

#include "TrackVehicle.h"

#include "subsys/trackSystem/TrackSystem.h"

#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


static ChVector<> loadVector(const Value& a)
{
  assert(a.IsArray());
  assert(a.Size() == 3);
  return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

static ChQuaternion<> loadQuaternion(const Value& a)
{
  assert(a.IsArray());
  assert(a.Size() == 4);
  return ChQuaternion<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble(), a[3u].GetDouble());
}


void TrackVehicle::Load_TrackSystem(const std::string& filename, int track)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  assert(d.HasMember("Name"));
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  
  m_TrackSystems[track] = ChSharedPtr<TrackSystem>(new TrackSystem(d));
  
  
}


TrackVehicle::TrackVehicle(const std::string& filename,
							bool               fixed)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  
  m_chassis = ChSharedPtr<ChBodyAuxRef>(new ChBodyAuxRef);

  m_Mass = d["Chassis"]["Mass"].GetDouble();
  m_COM = loadVector(d["Chassis"]["COM"]);
  m_Inertia = loadVector(d["Chassis"]["Inertia"]);

  m_chassis->SetIdentifier(0);
  m_chassis->SetName("chassis");
  m_chassis->SetMass(m_chassisMass);
  m_chassis->SetFrame_COG_to_REF(ChFrame<>(m_chassisCOM, ChQuaternion<>(1, 0, 0, 0)));
  m_chassis->SetInertiaXX(m_chassisInertia);
  m_chassis->SetBodyFixed(fixed);

  if (d.HasMember("Visualization"))
  {
    assert(d["Visualization"].HasMember("Filename"));
    assert(d["Visualization"].HasMember("Name"));

    m_chassisMeshFile = d["Visualization"]["Filename"].GetString();
    m_chassisMeshName = d["Visualization"]["Name"].GetString();

    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(utils::GetModelDataFile(m_chassisMeshFile), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_chassisMeshName);
    m_chassis->AddAsset(trimesh_shape);

    m_chassisUseMesh = true;
  }
  else
  {
    ChSharedPtr<ChSphereShape> sphere(new ChSphereShape);
    sphere->GetSphereGeometry().rad = 0.1;
    sphere->Pos = m_chassisCOM;
    m_chassis->AddAsset(sphere);
  }

  Add(m_chassis);


  assert(d.HasMember("TrackSystem"));
  assert(d.HasMember("Tracks"));

  m_num_tracks = d["TrackSystem"].Size();

  m_TrackSystems.resize(m_num_tracks);
  m_TrackSystem_locs.resize(m_num_tracks);

  // create track systems
  for (int i = 0; i < m_num_tracks; i++) {
    std::string file_name = d["TrackSystem"][i]["Input File"].GetString();
    Load_TrackSystem(utils::GetModelDataFile(file_name), i);
    m_TrackSystem_locs[i] = loadVector(d["Axles"][i]["Location"]);

  }

  m_driverCsys.pos = loadVector(d["Driver Position"]["Location"]);
  m_driverCsys.rot = loadQuaternion(d["Driver Position"]["Orientation"]);
}



void TrackVehicle::Initialize(const ChCoordsys<>& chassisPos)
{
  m_chassis->SetFrame_REF_to_abs(ChFrame<>(chassisPos));

  // Initialize the track systems

  // Initialize the suspension, wheel, and brake subsystems.
  for (int i = 0; i < m_num_tracks; i++)
  {
    m_TrackSystems[i]->Initialize(m_chassis, m_TrackSystem_locs[i];
  }
}


void TrackVehicle::Update(double	time,
                     double	left_drive_input,
					 double	right_drive_input);
{
 
  // apply sprocket inputs, left = 0, right = 1
  m_TrackSystems[LEFT]->ApplyTireForce(left_drive_input);
  m_TrackSystems[RIGHT]->ApplyTireForce(right_drive_input);
}


} // end namespace chrono
