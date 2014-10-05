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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Vehicle wheel constructed with data from file (JSON format).
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"
#include "physics/ChGlobal.h"

#include "subsys/wheel/Wheel.h"
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
Wheel::Wheel(const std::string& filename)
: m_vis(NONE)
{
  FILE* fp = fopen(filename.c_str(), "r");

  char readBuffer[65536];
  FileReadStream is(fp, readBuffer, sizeof(readBuffer));

  fclose(fp);

  Document d;
  d.ParseStream(is);

  Create(d);
}

Wheel::Wheel(const Document& d)
: m_vis(NONE)
{
  Create(d);
}

void Wheel::Create(const Document& d)
{
  // Read top-level data
  assert(d.HasMember("Type"));
  assert(d.HasMember("Template"));
  assert(d.HasMember("Name"));

  // Read mass and inertia
  m_mass = d["Mass"].GetDouble();
  m_inertia = loadVector(d["Inertia"]);

  // Check how to visualize this wheel.
  if (d.HasMember("Visualization")) {
    if (d["Visualization"].HasMember("Mesh Filename")) {
      m_meshFile = d["Visualization"]["Mesh Filename"].GetString();
      m_meshName = d["Visualization"]["Mesh Name"].GetString();
      m_vis = MESH;
    }
    else {
      m_radius = d["Visualization"]["Radius"].GetDouble();
      m_width = d["Visualization"]["Width"].GetDouble();
      m_vis = PRIMITIVES;
    }
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Wheel::Initialize(ChSharedBodyPtr spindle)
{
  // Call the base class initialization function
  ChWheel::Initialize(spindle);

  // Attach visualization
  switch (m_vis) {
  case PRIMITIVES:
  {
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().rad = m_radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, m_width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -m_width / 2, 0);
    spindle->AddAsset(cyl);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    spindle->AddAsset(tex);

    break;
  }
  case MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(utils::GetModelDataFile(m_meshFile), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_meshName);
    spindle->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    spindle->AddAsset(mcolor);

    break;
  }
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Wheel::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_meshFile, m_meshName, out_dir, ChColor(0.15f, 0.15f, 0.15f));
}


}  // end namespace chrono
