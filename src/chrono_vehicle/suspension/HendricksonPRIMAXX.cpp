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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Hendrickson PRIMAXX suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/suspension/HendricksonPRIMAXX.h"

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
// Construct a double wishbone suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
HendricksonPRIMAXX::HendricksonPRIMAXX(const std::string& filename)
: ChHendricksonPRIMAXX("")
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

HendricksonPRIMAXX::HendricksonPRIMAXX(const rapidjson::Document& d)
: ChHendricksonPRIMAXX("")
{
  Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HendricksonPRIMAXX::~HendricksonPRIMAXX()
{
}

// -----------------------------------------------------------------------------
// Worker function for creating a HendricksonPRIMAXX suspension using data in
// the specified RapidJSON document.
// -----------------------------------------------------------------------------
void HendricksonPRIMAXX::Create(const rapidjson::Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  SetName(d["Name"].GetString());

  // Read Spindle data
  assert(d.HasMember("Spindle"));
  assert(d["Spindle"].IsObject());

  m_spindleMass = d["Spindle"]["Mass"].GetDouble();
  m_points[SPINDLE] = loadVector(d["Spindle"]["COM"]);
  m_spindleInertia = loadVector(d["Spindle"]["Inertia"]);
  m_spindleRadius = d["Spindle"]["Radius"].GetDouble();
  m_spindleWidth = d["Spindle"]["Width"].GetDouble();

  //// TODO


  // Read axle inertia
  assert(d.HasMember("Axle"));
  assert(d["Axle"].IsObject());

  m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}


} // end namespace chrono
