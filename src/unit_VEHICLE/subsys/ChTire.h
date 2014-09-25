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
// A tire system is a force element. It is passed position and velocity
// information of the wheel body and it produces ground reaction forces and
// moments to be applied to the wheel body.
//
// =============================================================================

#ifndef CH_TIRE_H
#define CH_TIRE_H

#include "core/ChShared.h"
#include "core/ChVector.h"
#include "core/ChQuaternion.h"

#include "subsys/ChApiSubsys.h"
#include "subsys/ChVehicle.h"
#include "subsys/ChTerrain.h"

namespace chrono {

///
/// Base class for a tire system.
/// A tire subsystem is a force element. It is passed position and velocity
/// information of the wheel body and it produces ground reaction forces and
/// moments to be applied to the wheel body.
///
class CH_SUBSYS_API ChTire : public ChShared
{
public:

  ChTire(
    const ChTerrain& terrain  ///< [in] reference to the terrain system
    );
  virtual ~ChTire() {}

  /// Update the state of this tire system at the current time.
  /// The tire system is provided the current state of its associated wheel.
  virtual void Update(
    double               time,          ///< [in] current time
    const ChWheelState&  wheel_state    ///< [in] current state of associated wheel body
    ) {}

  /// Advance the state of this tire by the specified time step.
  virtual void Advance(double step) {}

  /// Get the tire force and moment.
  /// This represents the output from this tire system that is passed to the
  /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
  /// to the appropriate suspension subsystem which applies it as an external
  /// force one the wheel body.
  virtual ChTireForce GetTireForce() const = 0;

protected:

  /// Perform disc-terrain collision detection.
  /// This utility function checks for contact between a disc of specified 
  /// radius with given position and orientation (specified as the location of
  /// its center and a unit vector normal to the disc plane) and the terrain
  /// system associated with this tire. It returns true if the disc contacts the
  /// terrain and false otherwise.  If contact occurrs, it returns a coordinate
  /// system with the Z axis along the contact normal and the X axis along the
  /// "rolling" direction, as well as a positive penetration depth (i.e. the
  /// height below the terrain of the lowest point on the disc).
  bool  disc_terrain_contact(
    const ChVector<>& disc_center,    ///< [in] global location of the disc center
    const ChVector<>& disc_normal,    ///< [in] disc normal, expressed in the global frame
    double            disc_radius,    ///< [in] disc radius
    ChCoordsys<>&     contact,        ///< [out] contact coordinate system (relative to the global frame)
    double&           depth           ///< [out] penetration depth (positive if contact occurred)
    );

  const ChTerrain&  m_terrain;   ///< reference to the terrain system
};


} // end namespace chrono


#endif
