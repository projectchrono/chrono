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
// Solid axle suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "subsys/suspension/SolidAxle.h"

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
// Construct a solid axle suspension using data from the specified JSON
// file.
// -----------------------------------------------------------------------------
SolidAxle::SolidAxle(const std::string& filename,
                               bool               driven)
: ChSolidAxle("", driven)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

SolidAxle::SolidAxle(const Document& d,
                               bool            driven)
: ChSolidAxle("", driven)
{
  Create(d);
}

void SolidAxle::Create(const Document& d)
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

  // Read Knuckle data
  assert(d.HasMember("Knuckle"));
  assert(d["Knuckle"].IsObject());

  m_knuckleMass = d["Knuckle"]["Mass"].GetDouble();
  m_points[KNUCKLE_CM] = loadVector(d["Knuckle"]["COM"]);
  m_knuckleInertia = loadVector(d["Knuckle"]["Inertia"]);
  m_knuckleRadius = d["Knuckle"]["Radius"].GetDouble();
  m_points[KNUCKLE_L] = loadVector(d["Knuckle"]["Location Lower"]);
  m_points[KNUCKLE_U] = loadVector(d["Knuckle"]["Location Upper"]);

  // Read UL data
  assert(d.HasMember("Upper Link"));
  assert(d["Upper Link"].IsObject());

  m_ULMass = d["Upper Link"]["Mass"].GetDouble();
  m_points[UL_CM] = loadVector(d["Upper Link"]["COM"]);
  m_ULInertia = loadVector(d["Upper Link"]["Inertia"]);
  m_ULRadius = d["Upper Link"]["Radius"].GetDouble();
  m_points[UL_A] = loadVector(d["Upper Link"]["Location Axle"]);
  m_points[UL_C] = loadVector(d["Upper Link"]["Location Chassis"]);
  m_directions[UNIV_AXIS_LINK_U] = loadVector(d["Upper Link"]["Universal Joint Axis Link"]);
  m_directions[UNIV_AXIS_CHASSIS_U] = loadVector(d["Upper Link"]["Universal Joint Axis Chassis"]);

  // Read LL data
  assert(d.HasMember("Lower Link"));
  assert(d["Lower Link"].IsObject());

  m_LLMass = d["Lower Link"]["Mass"].GetDouble();
  m_points[LL_CM] = loadVector(d["Lower Link"]["COM"]);
  m_LLInertia = loadVector(d["Lower Link"]["Inertia"]);
  m_LLRadius = d["Lower Link"]["Radius"].GetDouble();
  m_points[LL_A] = loadVector(d["Lower Link"]["Location Axle"]);
  m_points[LL_C] = loadVector(d["Lower Link"]["Location Chassis"]);
  m_directions[UNIV_AXIS_LINK_L] = loadVector(d["Lower Link"]["Universal Joint Axis Link"]);
  m_directions[UNIV_AXIS_CHASSIS_L] = loadVector(d["Lower Link"]["Universal Joint Axis Chassis"]);

  // Read Axle Tube data
  assert(d.HasMember("Axle Tube"));
  assert(d["Axle Tube"].IsObject());

  m_axleTubeMass = d["Axle Tube"]["Mass"].GetDouble();
  m_axleTubeCOM = loadVector(d["Axle Tube"]["COM"]);
  m_axleTubeInertia = loadVector(d["Axle Tube"]["Inertia"]);
  m_axleTubeRadius = d["Axle Tube"]["Radius"].GetDouble();

  // Read Tierod data
  assert(d.HasMember("Tierod"));
  assert(d["Tierod"].IsObject());

  m_points[TIEROD_C] = loadVector(d["Tierod"]["Location Chassis"]);
  m_points[TIEROD_K] = loadVector(d["Tierod"]["Location Knuckle"]);

  // Read spring data
  assert(d.HasMember("Spring"));
  assert(d["Spring"].IsObject());

  m_points[SPRING_C] = loadVector(d["Spring"]["Location Chassis"]);
  m_points[SPRING_A] = loadVector(d["Spring"]["Location Axle"]);
  m_springCoefficient = d["Spring"]["Spring Coefficient"].GetDouble();
  m_springRestLength = d["Spring"]["Free Length"].GetDouble();

  // Read shock data
  assert(d.HasMember("Shock"));
  assert(d["Shock"].IsObject());

  m_points[SHOCK_C] = loadVector(d["Shock"]["Location Chassis"]);
  m_points[SHOCK_A] = loadVector(d["Shock"]["Location Axle"]);
  m_dampingCoefficient = d["Shock"]["Damping Coefficient"].GetDouble();

  // Read axle inertia
  if (IsDriven()) {
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());

    m_axleInertia = d["Axle"]["Inertia"].GetDouble();
  }
}


} // end namespace chrono