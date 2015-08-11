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
// RSD antirollbar model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/antirollbar/AntirollBarRSD.h"

#include "thirdparty/rapidjson/filereadstream.h"

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
AntirollBarRSD::AntirollBarRSD(const std::string& filename)
: ChAntirollBarRSD("")
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

AntirollBarRSD::AntirollBarRSD(const rapidjson::Document& d)
: ChAntirollBarRSD("")
{
  Create(d);
}

void AntirollBarRSD::Create(const rapidjson::Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  SetName(d["Name"].GetString());

  // Read arm data
  m_arm_mass = d["Arm"]["Mass"].GetDouble();
  m_arm_inertia = loadVector(d["Arm"]["Inertia"]);
  m_arm_length = d["Arm"]["Length"].GetDouble();
  m_arm_width = d["Arm"]["Width"].GetDouble();
  m_arm_radius = d["Arm"]["Radius"].GetDouble();

  // Read droplink data
  m_link_height = d["Droplink"]["Height"].GetDouble();

  // Read RSD data
  m_spring_coef = d["RSD"]["Spring Coefficient"].GetDouble();
  m_damping_coef = d["RSD"]["Damping Coefficient"].GetDouble();
}


}  // end namespace chrono
