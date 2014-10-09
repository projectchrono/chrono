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
// Simplified powertrain model constructed with data from file (JSON format).
//
// =============================================================================

#include "physics/ChGlobal.h"

#include "subsys/powertrain/SimplePowertrain.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

#include "rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
SimplePowertrain::SimplePowertrain(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

SimplePowertrain::SimplePowertrain(const rapidjson::Document& d)
{
  Create(d);
}

void SimplePowertrain::Create(const rapidjson::Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  // Read data
  m_fwd_gear_ratio = d["Forward Gear Ratio"].GetDouble();
  m_rev_gear_ratio = d["Reverse Gear Ratio"].GetDouble();

  m_max_torque = d["Maximum Engine Torque"].GetDouble();
  m_max_speed = d["Maximum Engine Speed"].GetDouble();
}


}  // end namespace chrono
