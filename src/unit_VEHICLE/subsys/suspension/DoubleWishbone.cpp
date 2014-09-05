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
// Double-A arm suspension constructed with data from file.
//
// =============================================================================

#include <cstdio>

#include "subsys/suspension/DoubleWishbone.h"

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
// Construct a double wishbone suspension using data from the specified JSON
// file.
//
// TODO: for now, we must always construct a driven suspension (i.e. we always
//       construct the axle shaft) since this is done in the ChDoubleWishbone
//       constructor.  Figure out a clean way of fixing this...  
// -----------------------------------------------------------------------------
DoubleWishbone::DoubleWishbone(const std::string& filename)
: ChDoubleWishbone("", false, true)
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
  assert(d.HasMember("Steerable"));
  assert(d.HasMember("Driven"));

  SetName(d["Name"].GetString());
  SetSteerable(d["Steerable"].GetBool());
  SetDriven(d["Driven"].GetBool());

  // Read Spindle data
  assert(d.HasMember("Spindle"));
  assert(d["Spindle"].IsObject());

  m_spindleMass = d["Spindle"]["mass"].GetDouble();
  m_points[SPINDLE] = loadVector(d["Spindle"]["com"]);
  m_spindleInertia = loadVector(d["Spindle"]["inertia"]);
  m_spindleRadius = d["Spindle"]["radius"].GetDouble();
  m_spindleWidth = d["Spindle"]["width"].GetDouble();

  // Read Upright data
  assert(d.HasMember("Upright"));
  assert(d["Upright"].IsObject());

  m_uprightMass = d["Upright"]["mass"].GetDouble();
  m_points[UPRIGHT] = loadVector(d["Upright"]["com"]);
  m_uprightInertia = loadVector(d["Upright"]["inertia"]);
  m_uprightRadius = d["Upright"]["radius"].GetDouble();

  // Read UCA data
  assert(d.HasMember("UCA"));
  assert(d["UCA"].IsObject());

  m_UCAMass = d["UCA"]["mass"].GetDouble();
  m_points[UCA_CM] = loadVector(d["UCA"]["com"]);
  m_UCAInertia = loadVector(d["UCA"]["inertia"]);
  m_UCARadius = d["UCA"]["radius"].GetDouble();
  m_points[UCA_F] = loadVector(d["UCA"]["point_chassis_F"]);
  m_points[UCA_B] = loadVector(d["UCA"]["point_chassis_B"]);
  m_points[UCA_U] = loadVector(d["UCA"]["point_upright"]);

  // Read LCA data
  assert(d.HasMember("LCA"));
  assert(d["LCA"].IsObject());

  m_LCAMass = d["LCA"]["mass"].GetDouble();
  m_points[LCA_CM] = loadVector(d["LCA"]["com"]);
  m_LCAInertia = loadVector(d["LCA"]["inertia"]);
  m_LCARadius = d["LCA"]["radius"].GetDouble();
  m_points[LCA_F] = loadVector(d["LCA"]["point_chassis_F"]);
  m_points[LCA_B] = loadVector(d["LCA"]["point_chassis_B"]);
  m_points[LCA_U] = loadVector(d["LCA"]["point_upright"]);

  // Read Tierod data
  assert(d.HasMember("Tierod"));
  assert(d["Tierod"].IsObject());

  m_points[TIEROD_C] = loadVector(d["Tierod"]["point_chassis"]);
  m_points[TIEROD_U] = loadVector(d["Tierod"]["point_upright"]);

  // Read spring-damper data
  assert(d.HasMember("TSDA"));
  assert(d["TSDA"].IsObject());

  m_points[SHOCK_C] = loadVector(d["TSDA"]["point_shock_C"]);
  m_points[SHOCK_A] = loadVector(d["TSDA"]["point_shock_A"]);
  m_points[SPRING_C] = loadVector(d["TSDA"]["point_spring_C"]);
  m_points[SPRING_A] = loadVector(d["TSDA"]["point_spring_A"]);
  m_springCoefficient = d["TSDA"]["spring_coef"].GetDouble();
  m_dampingCoefficient = d["TSDA"]["damping_coef"].GetDouble();
  m_springRestLength = d["TSDA"]["free_length"].GetDouble();

  // Read axle inertia
  if (IsDriven()) {
    assert(d.HasMember("Axle"));
    assert(d["Axle"].IsObject());

    m_axleInertia = d["Axle"]["inertia"].GetDouble();
  }
}


} // end namespace chrono