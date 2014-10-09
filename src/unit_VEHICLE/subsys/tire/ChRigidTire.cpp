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
// Generic rigid tire
//
// =============================================================================


#include "ChRigidTire.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChRigidTire::ChRigidTire(const std::string& name,
                         const ChTerrain&   terrain)
: ChTire(name, terrain)
{
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChRigidTire::Initialize(ChSharedBodyPtr wheel)
{
  wheel->SetCollide(true);

  wheel->GetCollisionModel()->ClearModel();
  wheel->GetCollisionModel()->AddCylinder(getRadius(), getRadius(), getWidth() / 2);
  wheel->GetCollisionModel()->BuildModel();

  wheel->GetMaterialSurface()->SetFriction(getFrictionCoefficient());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTireForce ChRigidTire::GetTireForce() const
{
  ChTireForce tire_force;

  tire_force.force = ChVector<>(0, 0, 0);
  tire_force.point = ChVector<>(0, 0, 0);
  tire_force.moment = ChVector<>(0, 0, 0);

  return tire_force;
}


} // end namespace chrono
