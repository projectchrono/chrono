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
// 4WD driveline model template based on ChShaft objects using data from file
// (JSON format).
//
// =============================================================================

#include "chrono_vehicle/driveline/ShaftsDriveline4WD.h"

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
ShaftsDriveline4WD::ShaftsDriveline4WD(const std::string& filename)
: ChShaftsDriveline4WD()
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

ShaftsDriveline4WD::ShaftsDriveline4WD(const rapidjson::Document& d)
: ChShaftsDriveline4WD()
{
  Create(d);
}

void ShaftsDriveline4WD::Create(const rapidjson::Document& d)
{
  // Read top-level data.
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  // Get shaft directions.
  assert(d.HasMember("Shaft Direction"));
  SetMotorBlockDirection(loadVector(d["Shaft Direction"]["Motor Block"]));
  SetAxleDirection(loadVector(d["Shaft Direction"]["Axle"]));

  // Read shaft inertias.
  assert(d.HasMember("Shaft Inertia"));
  m_driveshaft_inertia = d["Shaft Inertia"]["Driveshaft"].GetDouble();
  m_frontshaft_inertia = d["Shaft Inertia"]["Front Driveshaft"].GetDouble();
  m_rearshaft_inertia = d["Shaft Inertia"]["Rear Driveshaft"].GetDouble();
  m_central_differentialbox_inertia = d["Shaft Inertia"]["Central Differential Box"].GetDouble();
  m_front_differentialbox_inertia = d["Shaft Inertia"]["Front Differential Box"].GetDouble();
  m_rear_differentialbox_inertia = d["Shaft Inertia"]["Rear Differential Box"].GetDouble();

  // Read gear ratios.
  assert(d.HasMember("Gear Ratio"));
  m_front_conicalgear_ratio = d["Gear Ratio"]["Front Conical Gear"].GetDouble();
  m_rear_conicalgear_ratio = d["Gear Ratio"]["Rear Conical Gear"].GetDouble();
  m_central_differential_ratio = d["Gear Ratio"]["Central Differential"].GetDouble();
  m_front_differential_ratio = d["Gear Ratio"]["Front Differential"].GetDouble();
  m_rear_differential_ratio = d["Gear Ratio"]["Rear Differential"].GetDouble();
}


} // end namespace chrono
