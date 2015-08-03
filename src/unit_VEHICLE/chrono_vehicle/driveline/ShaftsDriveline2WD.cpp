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
// 2WD driveline model template based on ChShaft objects using data from file
// (JSON format).
//
// =============================================================================

#include "subsys/driveline/ShaftsDriveline2WD.h"

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
ShaftsDriveline2WD::ShaftsDriveline2WD(const std::string& filename)
: ChShaftsDriveline2WD()
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

ShaftsDriveline2WD::ShaftsDriveline2WD(const rapidjson::Document& d)
: ChShaftsDriveline2WD()
{
  Create(d);
}

void ShaftsDriveline2WD::Create(const rapidjson::Document& d)
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
  m_differentialbox_inertia = d["Shaft Inertia"]["Differential Box"].GetDouble();

  // Read gear ratios.
  assert(d.HasMember("Gear Ratio"));
  m_conicalgear_ratio = d["Gear Ratio"]["Conical Gear"].GetDouble();
  m_differential_ratio = d["Gear Ratio"]["Differential"].GetDouble();
}


} // end namespace chrono
