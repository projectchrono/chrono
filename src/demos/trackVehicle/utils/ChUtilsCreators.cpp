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
// A set of utility functions for adding geometry (collision and visualization)
// to ChBody objects.
//
// =============================================================================


#include "ChUtilsCreators.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// CreateBoxContainer
// CreateBoxContainerDEM
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// These functions assume a coordinate frame with Z up.
// The first version uses a ChBody; the second version uses a ChBodyDEM.
// -----------------------------------------------------------------------------
void CreateBoxContainer(ChSystem*                           system,
                        int                                 id,
                        ChSharedPtr<ChMaterialSurface>&     mat,
                        const ChVector<>&                   hdim,
                        double                              hthick,
                        const ChVector<>&                   pos,
                        const ChQuaternion<>&               rot,
                        bool                                collide)
{
  // Create the body and set material
  ChBody* body;

  body = new ChBody();
  body->SetMaterialSurface(mat);

  // Set body properties and geometry.
  body->SetIdentifier(id);
  body->SetMass(1);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(true);

  body->GetCollisionModel()->ClearModel();
  AddBoxGeometry(body, ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -hthick));
  AddBoxGeometry(body, ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(-hdim.x-hthick, 0, hdim.z));
  AddBoxGeometry(body, ChVector<>(hthick, hdim.y, hdim.z), ChVector<>( hdim.x+hthick, 0, hdim.z));
  AddBoxGeometry(body, ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, -hdim.y-hthick, hdim.z));
  AddBoxGeometry(body, ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0,  hdim.y+hthick, hdim.z));
  body->GetCollisionModel()->BuildModel();

  // Attach the body to the system.
  system->AddBody(ChSharedPtr<ChBody>(body));
}

void CreateBoxContainerDEM(ChSystem*                           system,
                           int                                 id,
                           ChSharedPtr<ChMaterialSurfaceDEM>&  mat,
                           const ChVector<>&                   hdim,
                           double                              hthick,
                           const ChVector<>&                   pos,
                           const ChQuaternion<>&               rot,
                           bool                                collide)
{
  // Create the body and set material
  ChBodyDEM* body;

  body = new ChBodyDEM();
  body->SetMaterialSurfaceDEM(mat);

  // Set body properties and geometry.
  body->SetIdentifier(id);
  body->SetMass(1);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(true);

  body->GetCollisionModel()->ClearModel();
  AddBoxGeometry(body, ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -hthick));
  AddBoxGeometry(body, ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(-hdim.x-hthick, 0, hdim.z));
  AddBoxGeometry(body, ChVector<>(hthick, hdim.y, hdim.z), ChVector<>( hdim.x+hthick, 0, hdim.z));
  AddBoxGeometry(body, ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, -hdim.y-hthick, hdim.z));
  AddBoxGeometry(body, ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0,  hdim.y+hthick, hdim.z));
  body->GetCollisionModel()->BuildModel();

  // Attach the body to the system.
  system->AddBody(ChSharedPtr<ChBodyDEM>(body));
}


}  // namespace utils
}  // namespace chrono