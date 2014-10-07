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
// Base class for a vehicle system.
//
// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
// pointing forward, and Y-axis towards the left of the vehicle.
//
// =============================================================================

#ifndef CH_VEHICLE_H
#define CH_VEHICLE_H

#include <vector>

#include "core/ChVector.h"
#include "physics/ChSystem.h"
#include "physics/ChBodyAuxRef.h"

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

// Forward declaration.
class ChDriveline;

///
/// Base class for chrono vehicle systems.
/// This class provides the interface between the vehicle system and other
/// systems (tires, driver, etc.)
///
class CH_SUBSYS_API ChVehicle : public ChSystem
{
public:

  ChVehicle();
  virtual ~ChVehicle() {}

  /// Get a handle to the vehicle's chassis body.
  const ChSharedPtr<ChBodyAuxRef> GetChassis() const { return m_chassis; }

  /// Get a handle to the vehicle's driveshaft body.
  const ChSharedPtr<ChShaft> GetDriveshaft() const;

  /// Get the global location of the chassis reference frame origin.
  const ChVector<>& GetChassisPos() const { return m_chassis->GetFrame_REF_to_abs().GetPos(); }

  /// Get the orientation of the chassis reference frame.
  /// The chassis orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetChassisRot() const { return m_chassis->GetFrame_REF_to_abs().GetRot(); }

  /// Get the global location of the chassis center of mass.
  const ChVector<>& GetChassisPosCOM() const { return m_chassis->GetPos(); }

  /// Get the orientation of the chassis centroidal frame.
  /// The chassis orientation is returned as a quaternion representing a
  /// rotation with respect to the global reference frame.
  const ChQuaternion<>& GetChassisRotCOM() const { return m_chassis->GetRot(); }

  /// Get the vehicle speed.
  /// Return the speed measured at the origin of the chassis reference frame.
  double GetVehicleSpeed() const { return m_chassis->GetFrame_REF_to_abs().GetPos_dt().Length(); }

  /// Get the speed of the chassis COM.
  /// Return the speed measured at the chassis center of mass.
  double GetVehicleSpeedCOM() const { return m_chassis->GetPos_dt().Length(); }

  /// Return the number of axles for this vehicle.
  virtual int GetNumberAxles() const = 0;

  /// Get a handle to the specified wheel body.
  virtual ChSharedPtr<ChBody> GetWheelBody(const ChWheelID& wheelID) const = 0;

  /// Get the global location of the specified wheel.
  virtual const ChVector<>& GetWheelPos(const ChWheelID& wheel_id) const = 0;

  /// Get the orientation of the specified wheel.
  /// The wheel orientation is returned as a quaternion representing a rotation
  /// with respect to the global reference frame.
  virtual const ChQuaternion<>& GetWheelRot(const ChWheelID& wheel_id) const = 0;

  /// Get the linear velocity of the specified wheel.
  /// Return the linear velocity of the wheel center, expressed in the global
  /// reference frame.
  virtual const ChVector<>& GetWheelLinVel(const ChWheelID& wheel_id) const = 0;

  /// Get the angular velocity of the specified wheel.
  /// Return the angular velocity of the wheel frame, expressed in the global
  /// reference frame.
  virtual ChVector<> GetWheelAngVel(const ChWheelID& wheel_id) const = 0;

  /// Get the angular speed of the specified wheel.
  /// This is the angular speed of the wheel axle.
  virtual double GetWheelOmega(const ChWheelID& wheel_id) const = 0;

  /// Get the complete state for the specified wheel.
  /// This includes the location, orientation, linear and angular velocities,
  /// all expressed in the global reference frame, as well as the wheel angular
  /// speed about its rotation axis.
  ChWheelState GetWheelState(const ChWheelID& wheel_id) const;

  /// Get the angular speed of the driveshaft.
  /// This function provides the interface between a vehicle system and a
  /// powertrain system.
  double GetDriveshaftSpeed() const;

  /// Get the local driver position and orientation.
  /// This is a coordinate system relative to the chassis reference frame.
  virtual ChCoordsys<> GetLocalDriverCoordsys() const = 0;

  /// Get the global location of the driver.
  ChVector<> GetDriverPos() const;

  /// Initialize this vehicle at the specified global location and orientation.
  virtual void Initialize(
    const ChCoordsys<>& chassisPos  ///< [in] initial global position and orientation
    ) {}

  /// Update the state of this vehicle at the current time.
  /// The vehicle system is provided the current driver inputs (throttle between
  /// 0 and 1, steering between -1 and +1, braking between 0 and 1), the torque
  /// from the powertrain, and tire forces (expressed in the global reference
  /// frame).
  virtual void Update(
    double              time,               ///< [in] current time
    double              steering,           ///< [in] current steering input [-1,+1]
    double              braking,            ///< [in] current braking input [0,1]
    double              powertrain_torque,  ///< [in] input torque from powertrain
    const ChTireForces& tire_forces         ///< [in] vector of tire force structures
    ) {}

  /// Advance the state of this vehicle by the specified time step.
  virtual void Advance(double step);

  /// Set the integration step size for the vehicle subsystem.
  void SetStepsize(double val) { m_stepsize = val; }

  /// Get the current value of the integration step size for the vehicle
  /// subsystem.
  double GetStepsize() const { return m_stepsize; }

  /// Log current constraint violations.
  virtual void LogConstraintViolations() {}

protected:

  ChSharedPtr<ChBodyAuxRef>  m_chassis;    ///< handle to the chassis body

  ChDriveline*               m_driveline;  ///< pointer to the driveline subsystem

  double                     m_stepsize;   ///< integration step-size for the vehicle system

  friend class ChDriveline;
  friend class ChIrrGuiDriver;
};


} // end namespace chrono


#endif
