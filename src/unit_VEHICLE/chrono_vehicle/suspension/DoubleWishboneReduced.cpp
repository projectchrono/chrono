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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Reduced double-A arm suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "chrono_vehicle/suspension/DoubleWishboneReduced.h"

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
// Construct a reduced double wishbone suspension using data from the specified
// JSON file.
// -----------------------------------------------------------------------------
DoubleWishboneReduced::DoubleWishboneReduced(const std::string& filename)
: ChDoubleWishboneReduced(""),
  m_shockForceCB(NULL)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

DoubleWishboneReduced::DoubleWishboneReduced(const rapidjson::Document& d)
: ChDoubleWishboneReduced(""),
  m_shockForceCB(NULL)
{
  Create(d);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
DoubleWishboneReduced::~DoubleWishboneReduced()
{
  delete m_shockForceCB;
}

// -----------------------------------------------------------------------------
// Worker function for creating a DoubleWishboneReduced suspension using data in
// the specified RapidJSON document.
// -----------------------------------------------------------------------------
void DoubleWishboneReduced::Create(const rapidjson::Document& d)
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

  // Read Upright data
  assert(d.HasMember("Upright"));
  assert(d["Upright"].IsObject());

  m_uprightMass = d["Upright"]["Mass"].GetDouble();
  m_points[UPRIGHT] = loadVector(d["Upright"]["COM"]);
  m_uprightInertia = loadVector(d["Upright"]["Inertia"]);
  m_uprightRadius = d["Upright"]["Radius"].GetDouble();

  // Read UCA data
  assert(d.HasMember("Upper Control Arm"));
  assert(d["Upper Control Arm"].IsObject());

  m_points[UCA_F] = loadVector(d["Upper Control Arm"]["Location Chassis Front"]);
  m_points[UCA_B] = loadVector(d["Upper Control Arm"]["Location Chassis Back"]);
  m_points[UCA_U] = loadVector(d["Upper Control Arm"]["Location Upright"]);

  // Read LCA data
  assert(d.HasMember("Lower Control Arm"));
  assert(d["Lower Control Arm"].IsObject());

  m_points[LCA_F] = loadVector(d["Lower Control Arm"]["Location Chassis Front"]);
  m_points[LCA_B] = loadVector(d["Lower Control Arm"]["Location Chassis Back"]);
  m_points[LCA_U] = loadVector(d["Lower Control Arm"]["Location Upright"]);

  // Read Tierod data
  assert(d.HasMember("Tierod"));
  assert(d["Tierod"].IsObject());

  m_points[TIEROD_C] = loadVector(d["Tierod"]["Location Chassis"]);
  m_points[TIEROD_U] = loadVector(d["Tierod"]["Location Upright"]);

  // Read spring-damper data and create force callback
  assert(d.HasMember("Shock"));
  assert(d["Shock"].IsObject());

  m_points[SHOCK_C] = loadVector(d["Shock"]["Location Chassis"]);
  m_points[SHOCK_U] = loadVector(d["Shock"]["Location Arm"]);
  m_springRestLength = d["Shock"]["Free Length"].GetDouble();
  m_shockForceCB = new LinearSpringDamperForce(d["Shock"]["Spring Coefficient"].GetDouble(),
                                               d["Shock"]["Damping Coefficient"].GetDouble());

  // Read axle inertia
  assert(d.HasMember("Axle"));
  assert(d["Axle"].IsObject());

  m_axleInertia = d["Axle"]["Inertia"].GetDouble();
}


} // end namespace chrono
