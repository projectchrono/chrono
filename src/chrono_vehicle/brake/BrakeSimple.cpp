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
// Vehicle simple brake model constructed with data from file (JSON format).
//
// =============================================================================

#include "chrono_vehicle/brake/BrakeSimple.h"

#include "thirdparty/rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
BrakeSimple::BrakeSimple(const std::string& filename)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

BrakeSimple::BrakeSimple(const rapidjson::Document& d)
{
  Create(d);
}

void BrakeSimple::Create(const rapidjson::Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  // Read maximum braking torque
  m_maxtorque = d["Maximum Torque"].GetDouble();
}


}  // end namespace chrono
