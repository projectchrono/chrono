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
// Authors: Radu Serban
// =============================================================================
//
// Pitman arm steering model constructed with data from file (JSON format).
//
// =============================================================================

#include "subsys/steering/PitmanArm.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


// -----------------------------------------------------------------------------
// This utility function returns a ChVector from the specified JSON array
// -----------------------------------------------------------------------------
static ChVector<> loadVector(const Value& a)
{
  assert(a.IsArray());
  assert(a.Size() == 3);

  return ChVector<>(a[0u].GetDouble(), a[1u].GetDouble(), a[2u].GetDouble());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
PitmanArm::PitmanArm(const std::string& filename)
: ChPitmanArm("")
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  SetName(d["Name"].GetString());

  // Read steering link data
  m_steeringLinkMass = d["Steering Link"]["mass"].GetDouble();
  m_points[STEERINGLINK] = loadVector(d["Steering Link"]["com"]);
  m_steeringLinkInertia = loadVector(d["Steering Link"]["inertia"]);
  m_steeringLinkRadius = d["Steering Link"]["radius"].GetDouble();

  // Read Pitman arm data
  m_pitmanArmMass = d["Pitman Arm"]["mass"].GetDouble();
  m_points[PITMANARM] = loadVector(d["Pitman Arm"]["com"]);
  m_pitmanArmInertia = loadVector(d["Pitman Arm"]["inertia"]);
  m_pitmanArmRadius = d["Pitman Arm"]["radius"].GetDouble();

  // Read data for the revolute joint (Pitman arm - chassis)
  m_points[REV] = loadVector(d["Revolute Joint"]["Location"]);
  m_dirs[REV_AXIS] = loadVector(d["Revolute Joint"]["Direction"]);

  // Read data for the universal joint (Pitman arm - steering link)
  m_points[UNIV] = loadVector(d["Universal Joint"]["Location"]);
  m_dirs[UNIV_AXIS_ARM] = loadVector(d["Universal Joint"]["Direction Arm"]);
  m_dirs[UNIV_AXIS_LINK] = loadVector(d["Universal Joint"]["Direction Link"]);

  // Read data for the revolute-spherical joint (chassis - steering link)
  m_points[REVSPH_R] = loadVector(d["Revolute-Spherical Joint"]["Location Chassis"]);
  m_points[REVSPH_S] = loadVector(d["Revolute-Spherical Joint"]["Location Link"]);
  m_dirs[REVSPH_AXIS] = loadVector(d["Revolute-Spherical Joint"]["Direction"]);

  // Read data for tireod connection points
  m_points[TIEROD_PA] = loadVector(d["Tierod Locations"]["Pitman Side"]);
  m_points[TIEROD_IA] = loadVector(d["Tierod Locations"]["Idler Side"]);
}


}  // end namespace chrono
