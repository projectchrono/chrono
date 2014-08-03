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

#include "HMMWV9_Wheel.h"
#include "HMMWV9_Vehicle.h"


using namespace chrono;


// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

static const double in2m = 0.0254;

const double      HMMWV9_Wheel::m_radius = 18.5 * in2m;
const double      HMMWV9_Wheel::m_width = 10 * in2m;

const double      HMMWV9_Wheel::m_mass = 54.7;
const ChVector<>  HMMWV9_Wheel::m_inertia(3.7958, 7.0037, 3.7958);

const std::string HMMWV9_Wheel::m_meshName = "hmmwv_tire";
const std::string HMMWV9_Wheel::m_meshFile = "../data/wheel_centered_rotated.obj";


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
HMMWV9_Wheel::HMMWV9_Wheel(bool   enableContact,
                           double mu,
                           int    visType)
: m_contact(enableContact),
  m_mu(mu),
  m_visType(visType)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void HMMWV9_Wheel::OnInitialize(ChSharedBodyPtr body)
{
  switch (m_visType) {
  case HMMWV9_Vehicle::PRIMITIVES:
  {
    ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
    cyl->GetCylinderGeometry().rad = m_radius;
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, m_width / 2, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0, -m_width / 2, 0);
    body->AddAsset(cyl);

    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename("../data/bluwhite.png");
    body->AddAsset(tex);

    break;
  }
  case HMMWV9_Vehicle::MESH:
  {
    geometry::ChTriangleMeshConnected trimesh;
    trimesh.LoadWavefrontMesh(m_meshFile, false, false);

    ChSharedPtr<ChTriangleMeshShape> trimesh_shape(new ChTriangleMeshShape);
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetName(m_meshName);
    body->AddAsset(trimesh_shape);

    break;
  }
  }

  body->SetCollide(m_contact);

  if (m_contact) {
    body->GetCollisionModel()->ClearModel();
    body->GetCollisionModel()->AddCylinder(m_radius, m_radius, m_width / 2);
    body->GetCollisionModel()->BuildModel();

    body->GetMaterialSurface()->SetFriction(m_mu);
  }
}
