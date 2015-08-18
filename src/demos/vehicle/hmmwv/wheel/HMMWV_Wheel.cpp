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
#include "assets/ChColorAsset.h"

#include "utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "hmmwv/wheel/HMMWV_Wheel.h"

using namespace chrono;

namespace hmmwv {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;
static const double lb2kg = 0.453592;

const double      HMMWV_Wheel::m_radius = in2m * 18.15;
const double      HMMWV_Wheel::m_width = in2m * 10;

const double      HMMWV_Wheel::m_mass = lb2kg * 100.00;
const ChVector<>  HMMWV_Wheel::m_inertia(0.113, 0.113, 0.113);

const std::string HMMWV_WheelLeft::m_meshName = "wheel_L_POV_geom";
const std::string HMMWV_WheelLeft::m_meshFile = vehicle::GetDataFile("hmmwv/wheel_L.obj");

const std::string HMMWV_WheelRight::m_meshName = "wheel_R_POV_geom";
const std::string HMMWV_WheelRight::m_meshFile = vehicle::GetDataFile("hmmwv/wheel_R.obj");


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV_Wheel::HMMWV_Wheel(VisualizationType  visType)
: m_visType(visType)
{
}

HMMWV_WheelLeft::HMMWV_WheelLeft(VisualizationType  visType)
: HMMWV_Wheel(visType)
{
}

HMMWV_WheelRight::HMMWV_WheelRight(VisualizationType  visType)
: HMMWV_Wheel(visType)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_Wheel::Initialize(ChSharedBodyPtr spindle)
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

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    spindle->AddAsset(mcolor);

    break;
  }
  }
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV_WheelLeft::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_meshFile, m_meshName, out_dir, ChColor(0.15f, 0.15f, 0.15f));
}

void HMMWV_WheelRight::ExportMeshPovray(const std::string& out_dir)
{
  utils::WriteMeshPovray(m_meshFile, m_meshName, out_dir, ChColor(0.15f, 0.15f, 0.15f));
}


} // end namespace hmmwv
