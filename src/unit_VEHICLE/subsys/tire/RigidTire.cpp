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
// Rigid tire constructed with data from file (JSON format).
//
// =============================================================================

#include "subsys/tire/RigidTire.h"

#include "utils/ChUtilsData.h"

#include "rapidjson/filereadstream.h"

using namespace rapidjson;

namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
RigidTire::RigidTire(const std::string&       filename,
                     const chrono::ChTerrain& terrain)
: ChRigidTire("", terrain)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

RigidTire::RigidTire(const rapidjson::Document& d,
                     const chrono::ChTerrain&   terrain)
: ChRigidTire("", terrain)
{
  Create(d);
}

void RigidTire::Create(const rapidjson::Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  SetName(d["Name"].GetString());

  m_mu = d["Coefficient of Friction"].GetDouble();
  m_radius = d["Radius"].GetDouble();
  m_width = d["Width"].GetDouble();
}


}  // end namespace chrono
