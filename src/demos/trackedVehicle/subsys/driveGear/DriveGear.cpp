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
// Authors: Justin Madsen
// =============================================================================
//
// The drive gear propels the tracked vehicle
//
// =============================================================================

#include "DriveGear.h"

#include "assets/ChCylinderShape.h"
#include "assets/ChTriangleMeshShape.h"
#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"

#include "utils/ChUtilsData.h"
#include "utils/ChUtilsInputOutput.h"

namespace chrono {

// static variables
const std::string DriveGear::m_meshName = "wheel_L_POV_geom";
const std::string DriveGear::m_meshFile = utils::GetModelDataFile("hmmwv/wheel_L.obj");

const double DriveGear::m_mass = 200;
const ChVector<> DriveGear::m_inertia(10,10,10);
const double DriveGear::m_radius = 0.3;
const double DriveGear::m_width = 0.25;


DriveGear::DriveGear(const std::string& name, VisualizationType vis, CollisionType collide)
{
  m_gear = ChSharedPtr<ChBody>(new ChBody);
}

void DriveGear::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& pos, const ChQuaternion<>& rot)
{

  // 0 = none, 1 = primitive, 2 = mesh
  m_visType = 0;

  // Attach visualization
  switch (m_visType) {
  case 1:
  {
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().rad = m_radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, m_width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -m_width / 2, 0);
    m_gear->AddAsset(cyl);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
    m_gear->AddAsset(tex);

    break;
  }
  case 2:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(getMeshFile(), false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(getMeshName());
    m_gear->AddAsset(trimesh_shape);

    ChSharedPtr<ChColorAsset> mcolor(new ChColorAsset(0.3f, 0.3f, 0.3f));
    m_gear->AddAsset(mcolor);

    break;
  }
  }
}

} // end namespace chrono
