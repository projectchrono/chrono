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
// Various utility classes for vehicle subsystems.
//
// =============================================================================

#ifndef CH_SUBSYS_DEFS_H
#define CH_SUBSYS_DEFS_H

#include <vector>

#include "core/ChVector.h"
#include "core/ChQuaternion.h"

#include "subsys/ChApiSubsys.h"

namespace chrono {

enum ChVehicleSide {
  LEFT = 0,     ///< left side of vehicle is always 0
  RIGHT = 1     ///< right side of vehicle is always 1
};

///
/// Class to encode the ID of a vehicle wheel.
/// By convention, wheels are counted front to rear and left to right. In other
/// words, for a vehicle with 2 axles, the order is: front-left, front-right,
/// rear-left, rear-right.
///
class ChWheelID
{
public:

  ChWheelID(int id) : m_id(id), m_axle(id / 2), m_side(ChVehicleSide(id % 2)) {}
  ChWheelID(int axle, ChVehicleSide side) : m_id(2 * axle + side), m_axle(axle), m_side(side) {}

  /// Return the wheel ID.
  int id() const { return m_id; }

  /// Return the axle index for this wheel ID.
  /// Axles are counted from the front of the vehicle.
  int axle() const { return m_axle; }

  /// Return the side for this wheel ID.
  /// By convention, left is 0 and right is 1.
  ChVehicleSide side() const { return m_side; }

private:

  int           m_id;     ///< wheel ID
  int           m_axle;   ///< axle index (counted from the front)
  ChVehicleSide m_side;   ///< vehicle side (LEFT: 0, RIGHT: 1)
};

/// Global constant wheel IDs for the common topology of a 2-axle vehicle.
static const ChWheelID  FRONT_LEFT(0, LEFT);
static const ChWheelID  FRONT_RIGHT(0, RIGHT);
static const ChWheelID  REAR_LEFT(1, LEFT);
static const ChWheelID  REAR_RIGHT(1, RIGHT);

///
/// Structure to communicate a full body state.
///
struct ChBodyState {
  ChVector<>     pos;      ///< global position
  ChQuaternion<> rot;      ///< orientation with respect to global frame
  ChVector<>     lin_vel;  ///< linear velocity, expressed in the global frame
  ChVector<>     ang_vel;  ///< angular velocity, expressed in the global frame
};

/// Vector of body state structures
typedef std::vector<ChBodyState> ChBodyStates;

///
/// Structure to communicate a fulle wheel body state.
/// In addition to the quantities communicated for a generic body, the wheel
/// state also includes the wheel angular speed about its axis of rotation.
///
struct ChWheelState {
  ChVector<>     pos;      ///< global position
  ChQuaternion<> rot;      ///< orientation with respect to global frame
  ChVector<>     lin_vel;  ///< linear velocity, expressed in the global frame
  ChVector<>     ang_vel;  ///< angular velocity, expressed in the global frame
  double         omega;    ///< wheel angular speed about its rotation axis
};

/// Vector of wheel state structures
typedef std::vector<ChWheelState> ChWheelStates;

///
/// Structure to communicate a set of generalized tire forces.
///
struct ChTireForce {
  ChVector<> force;        ///< force vector, epxressed in the global frame
  ChVector<> point;        ///< global location of the force application point
  ChVector<> moment;       ///< moment vector, expressed in the global frame
};

/// Vector of tire force structures.
typedef std::vector<ChTireForce> ChTireForces;


} // end namespace chrono


#endif
