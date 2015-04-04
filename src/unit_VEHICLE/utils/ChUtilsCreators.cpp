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


#include "utils/ChUtilsCreators.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
// CreateBoxContainer
//
// Create a fixed body with contact and asset geometry representing a box with 5
// walls (no top).
// These functions assume a coordinate frame with Z up.
// -----------------------------------------------------------------------------
void CreateBoxContainer(ChSystem*                           system,
                        int                                 id,
                        ChSharedPtr<ChMaterialSurfaceBase>  mat,
                        const ChVector<>&                   hdim,
                        double                              hthick,
                        const ChVector<>&                   pos,
                        const ChQuaternion<>&               rot,
                        bool                                collide)
{
  // Infer the type of contact method from the specified material properties.
  ChBody::ContactMethod contact_method = mat.IsType<ChMaterialSurface>() ? ChBody::DVI : ChBody::DEM;

  // Create the body and set material
  ChSharedPtr<ChBody> body(new ChBody(contact_method));

  body->SetMaterialSurface(mat);

  // Set body properties and geometry.
  body->SetIdentifier(id);
  body->SetMass(1);
  body->SetPos(pos);
  body->SetRot(rot);
  body->SetCollide(collide);
  body->SetBodyFixed(true);

  body->GetCollisionModel()->ClearModel();
  AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x, hdim.y, hthick), ChVector<>(0, 0, -hthick));
  AddBoxGeometry(body.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>(-hdim.x-hthick, 0, hdim.z));
  AddBoxGeometry(body.get_ptr(), ChVector<>(hthick, hdim.y, hdim.z), ChVector<>( hdim.x+hthick, 0, hdim.z));
  AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0, -hdim.y-hthick, hdim.z));
  AddBoxGeometry(body.get_ptr(), ChVector<>(hdim.x, hthick, hdim.z), ChVector<>(0,  hdim.y+hthick, hdim.z));
  body->GetCollisionModel()->BuildModel();

  // Attach the body to the system.
  system->AddBody(body);
}


}  // namespace utils
}  // namespace chrono