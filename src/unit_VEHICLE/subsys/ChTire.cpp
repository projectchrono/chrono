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
// Base class for a tire.
// A tire subsystem is a force element. It is passed position and velocity
// information of the wheel body and it produces ground reaction forces and
// moments to be applied to the wheel body.
//
// =============================================================================

#include "subsys/ChTire.h"


namespace chrono {


ChTire::ChTire(const ChTerrain& terrain)
: m_terrain(terrain)
{
}


// -----------------------------------------------------------------------------
// Utility function for characterizing the geometric contact between a disc with
// specified center location, normal direction, and radius and the terrain,
// assumed to be specified as a height field (over the x-y domain).
// This function returns false if no contact occurs. Otherwise, it sets the
// roll direction (a horizontal unit vector in the plane of the disc), the
// lowest (most penetrated) point on the disc, the corresponding terrain point,
// and the resulting penetration depth (positive).
// -----------------------------------------------------------------------------
bool ChTire::disc_terrain_contact(const ChVector<>& center,
                                  const ChVector<>& normal,
                                  double            radius,
                                  ChVector<>&       roll,
                                  ChVector<>&       disc_point,
                                  ChVector<>&       terrain_point,
                                  double&           depth)
{
  // Find terrain height below disc center. There is no contact if the disc
  // center is below the terrain or farther away by more than its radius.
  double hc = m_terrain.GetHeight(center.x, center.y);
  if (center.z <= hc || center.z >= hc + radius)
    return false;

  // Calculate the wheel roll direction, as the intersection between the disc
  // plane and the horizontal plane.
  roll = Vcross(normal, ChVector<>(0, 0, 1));

  // If the disc is (almost) horizontal, there is no contact.
  double sinTilt2 = roll.Length2();

  if (sinTilt2 < 1e-3)
    return false;

  // Calculate the lowest point on the disc.
  roll = roll / sqrt(sinTilt2);
  ChVector<> dir = Vcross(normal, roll);
  disc_point = center + dir * radius;

  // Calculate terrain height at lowest point. There is no contact if the lowest
  // point is above the terrain.
  double hp = m_terrain.GetHeight(disc_point.x, disc_point.y);
  depth = hp - disc_point.z;

  if (depth <= 0)
    return false;

  // The disc is in contact with the terrain.
  terrain_point.x = disc_point.x;
  terrain_point.y = disc_point.y;
  terrain_point.z = hp;

  return true;
}


}  // end namespace chrono
