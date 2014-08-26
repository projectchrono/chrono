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
: ChTire(terrain),
  m_force(0, 0, 0),
  m_point(0, 0, 0),
  m_moment(0, 0, 0)
{
}


// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLugreTire::Update(double              time,
                         const ChBodyState&  wheel_state)
{
  int numDiscs = getNumDiscs();
  double radius = getRadius();
  const double* discLocs = getDiscLocations();

  // Clear the force accumulators and set the application point to the wheel
  // center.
  m_force = ChVector<>(0, 0, 0);
  m_moment = ChVector<>(0, 0, 0);
  m_point = wheel_state.pos;

  // Extract the wheel normal (expressed in global frame)
  ChMatrix33<> A(wheel_state.rot);
  ChVector<> normal = A.Get_A_Yaxis();

  // Loop over all discs, check contact with terrain and accumulate normal tire
  // forces.
  ChVector<> roll;
  ChVector<> disc_point;
  ChVector<> terrain_point;
  double depth;

  for (int id = 0; id < numDiscs; id++) {
    // Calculate center of disk (expressed in global frame)
    ChVector<> center = wheel_state.pos + discLocs[id] * normal;

    // Check contact with terrain and calculate contact points.
    if (!disc_terrain_contact(center, normal, radius, roll, disc_point, terrain_point, depth))
      continue;

    // Generate normal contact force and add to accumulators.
    double relVelNormal_mag = wheel_state.lin_vel.z;

    //// TODO

  }

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

  tire_force.force = m_force;
  tire_force.point = m_point;
  tire_force.moment = m_moment;

  return tire_force;
}


} // end namespace chrono
