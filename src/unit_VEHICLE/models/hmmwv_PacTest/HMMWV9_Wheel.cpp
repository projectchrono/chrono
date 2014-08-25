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
// HMMWV wheel subsystem
//
// =============================================================================

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

#include "HMMWV9_Wheel.h"
#include "HMMWV9_Vehicle.h"

using namespace chrono;

namespace pactest {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double      HMMWV9_Wheel::m_radius = 18.5 * in2m;
const double      HMMWV9_Wheel::m_width = 10 * in2m;

const double      HMMWV9_Wheel::m_mass = 54.7;
const ChVector<>  HMMWV9_Wheel::m_inertia(3.7958, 7.0037, 3.7958);

const std::string HMMWV9_WheelLeft::m_meshName = "hmmwv_wheel_L";
const std::string HMMWV9_WheelLeft::m_meshFile = utils::GetModelDataFile("hmmwv/wheel_L.obj");

const std::string HMMWV9_WheelRight::m_meshName = "hmmwv_wheel_R";
const std::string HMMWV9_WheelRight::m_meshFile = utils::GetModelDataFile("hmmwv/wheel_R.obj");


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Wheel::HMMWV9_Wheel(VisualizationType  visType)
: m_visType(visType)
{
}

HMMWV9_WheelLeft::HMMWV9_WheelLeft(VisualizationType  visType)
: HMMWV9_Wheel(visType)
{
}

HMMWV9_WheelRight::HMMWV9_WheelRight(VisualizationType  visType)
: HMMWV9_Wheel(visType)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Wheel::Initialize(ChSharedBodyPtr spindle)
{
  // First, invoke the base class method
  ChWheel::Initialize(spindle);

  // Attach visualization
  switch (m_visType) {
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
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    spindle->AddAsset(trimesh_shape);

    break;
  }
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_WheelLeft::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_meshFile, m_meshName, out_dir);
}

void HMMWV9_WheelRight::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_meshFile, m_meshName, out_dir);
}


} // end namespace pactest
