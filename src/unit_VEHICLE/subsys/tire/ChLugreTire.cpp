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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for LuGre tire model
//
// =============================================================================


#include "ChLugreTire.h"


namespace chrono {


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChLugreTire::ChLugreTire(const ChTerrain& terrain)
: ChTire(terrain)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Initialize(ChSharedBodyPtr wheel)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Update(double              time,
                         const ChBodyState&  wheel_state)
{

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Advance(double step)
{

}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChTireForce ChLugreTire::GetTireForce() const
{
  ChTireForce tire_force;

  tire_force.force = ChVector<>(0, 0, 0);
  tire_force.point = ChVector<>(0, 0, 0);
  tire_force.moment = ChVector<>(0, 0, 0);

  return tire_force;
}


} // end namespace chrono
