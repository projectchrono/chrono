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
// LuGre tire constructed with data from file (JSON format).
//
// =============================================================================

#include "subsys/tire/LugreTire.h"
#include "utils/ChUtilsInputOutput.h"
#include "utils/ChUtilsData.h"

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
LugreTire::LugreTire(const std::string&       filename,
                     const chrono::ChTerrain& terrain)
: ChLugreTire("", terrain),
  m_discLocs(NULL)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

LugreTire::LugreTire(const rapidjson::Document& d,
                     const chrono::ChTerrain&   terrain)
: ChLugreTire("", terrain),
  m_discLocs(NULL)
{
  Create(d);
}

LugreTire::~LugreTire()
{
  delete m_discLocs;
}

void LugreTire::Create(const rapidjson::Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  SetName(d["Name"].GetString());

  // Read tire radius
  m_radius = d["Radius"].GetDouble();

  // Read disc locations
  m_numDiscs = d["Disc Locations"].Size();
  m_discLocs = new double[m_numDiscs];
  for (int i = 0; i < m_numDiscs; i++) {
    m_discLocs[i] = d["Disc Locations"][SizeType(i)].GetDouble();
  }

  // Read normal stiffness and damping
  m_normalStiffness = d["Normal Stiffness"].GetDouble();
  m_normalDamping = d["Normal Damping"].GetDouble();

  // Read LuGre model parameters
  m_sigma0[0] = d["Lugre Parameters"]["sigma0"][0u].GetDouble();  // longitudinal
  m_sigma0[1] = d["Lugre Parameters"]["sigma0"][1u].GetDouble();  // lateral

  m_sigma1[0] = d["Lugre Parameters"]["sigma1"][0u].GetDouble();  // longitudinal
  m_sigma1[1] = d["Lugre Parameters"]["sigma1"][1u].GetDouble();  // lateral

  m_sigma2[0] = d["Lugre Parameters"]["sigma2"][0u].GetDouble();  // longitudinal
  m_sigma2[1] = d["Lugre Parameters"]["sigma2"][1u].GetDouble();  // lateral

  m_Fc[0] = d["Lugre Parameters"]["Fc"][0u].GetDouble();  // longitudinal
  m_Fc[1] = d["Lugre Parameters"]["Fc"][1u].GetDouble();  // lateral

  m_Fs[0] = d["Lugre Parameters"]["Fs"][0u].GetDouble();  // longitudinal
  m_Fs[1] = d["Lugre Parameters"]["Fs"][1u].GetDouble();  // lateral

  m_vs[0] = d["Lugre Parameters"]["vs"][0u].GetDouble();  // longitudinal
  m_vs[1] = d["Lugre Parameters"]["vs"][1u].GetDouble();  // lateral
}



}  // end namespace chrono
