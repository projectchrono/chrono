// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
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

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle
/// @{

/// Base class for chrono vehicle systems.
/// The reference frame for a vehicle follows the ISO standard: Z-axis up, X-axis
/// pointing forward, and Y-axis towards the left of the vehicle.
class CH_VEHICLE_API ChVehicle {
  public:
    /// Destructor.
    virtual ~ChVehicle();

    /// Get the name identifier for this vehicle.
    const std::string& GetName() const { return m_name; }

    /// Set the name identifier for this vehicle.
    void SetName(const std::string& name) { m_name = name; }

    /// Get a pointer to the Chrono ChSystem.
    ChSystem* GetSystem() { return m_system; }

    /// Get the current simulation time of the underlying ChSystem.
    double GetChTime() const { return m_system->GetChTime(); }

    /// Get a handle to the vehicle's chassis subsystem.
    std::shared_ptr<ChChassis> GetChassis() const { return m_chassis; }

    /// Get a handle to the vehicle's chassis body.
    std::shared_ptr<ChBodyAuxRef> GetChassisBody() const { return m_chassis->GetBody(); }

    /// Get the vehicle total mass.
    /// This includes the mass of the chassis and all vehicle subsystems.
    virtual double GetVehicleMass() const = 0;

    /// Get the current global vehicle COM location.
    virtual ChVector<> GetVehicleCOMPos() const = 0;

    /// Get the vehicle location.
    /// This is the global location of the chassis reference frame origin.
    const ChVector<>& GetVehiclePos() const { return m_chassis->GetPos(); }
    
    /// Get the vehicle orientation.
    /// This is the chassis orientation, returned as a quaternion representing a
    /// rotation with respect to the global reference frame.
    const ChQuaternion<>& GetVehicleRot() const { return m_chassis->GetRot(); }

    /// Get the vehicle speed.
    /// Return the speed measured at the origin of the chassis reference frame.
    double GetVehicleSpeed() const { return m_chassis->GetSpeed(); }

    /// Get the speed of the chassis COM.
    /// Return the speed measured at the chassis center of mass.
    double GetVehicleSpeedCOM() const { return m_chassis->GetCOMSpeed(); }

    /// Get the global position of the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned location is expressed in the global reference frame.
    ChVector<> GetVehiclePointLocation(const ChVector<>& locpos) const { return m_chassis->GetPointLocation(locpos); }

    /// Get the global velocity of the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned velocity is expressed in the global reference frame.
    ChVector<> GetVehiclePointVelocity(const ChVector<>& locpos) const { return m_chassis->GetPointVelocity(locpos); }

    /// Get the acceleration at the specified point.
    /// The point is assumed to be given relative to the chassis reference frame.
    /// The returned acceleration is expressed in the chassis reference frame.
    ChVector<> GetVehicleAcceleration(const ChVector<>& locpos) const { return m_chassis->GetPointAcceleration(locpos); }

    /// Get a handle to the vehicle's driveshaft body.
    virtual std::shared_ptr<ChShaft> GetDriveshaft() const = 0;

    /// Get the angular speed of the driveshaft.
    /// This function provides the interface between a vehicle system and a
    /// powertrain system.
    virtual double GetDriveshaftSpeed() const = 0;

    /// Get the global location of the driver.
    ChVector<> GetDriverPos() const { return m_chassis->GetDriverPos(); }

    /// Initialize this vehicle at the specified global location and orientation.
    virtual void Initialize(const ChCoordsys<>& chassisPos,  ///< [in] initial global position and orientation
                            double chassisFwdVel = 0         ///< [in] initial chassis forward velocity
                            ) = 0;

    /// Set visualization mode for the chassis subsystem.
    void SetChassisVisualizationType(VisualizationType vis);

    /// Enable/disable collision for the chassis subsystem. This function controls
    /// contact of the chassis with all other collision shapes in the simulation.
    void SetChassisCollide(bool state);

    /// Enable/disable collision between the chassis and all other vehicle
    /// subsystems. Note that some of these collisions may be always disabled,
    /// as set by the particular derived vehicle class.
    virtual void SetChassisVehicleCollide(bool state) {}

    /// Advance the state of this vehicle by the specified time step.
    virtual void Advance(double step);

    /// Set the integration step size for the vehicle system.
    void SetStepsize(double val) { m_stepsize = val; }

    /// Get the current value of the integration step size for the vehicle system.
    double GetStepsize() const { return m_stepsize; }

    /// Log current constraint violations.
    virtual void LogConstraintViolations() = 0;

  protected:
    /// Construct a vehicle system with an underlying ChSystem.
    ChVehicle(const std::string& name,                                                  ///< [in] vehicle name
              ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC  ///< [in] contact method
              );

    /// Construct a vehicle system using the specified ChSystem.
    /// All physical components of the vehicle will be added to that system.
    ChVehicle(const std::string& name,  ///< [in] vehicle name
              ChSystem* system          ///< [in] containing mechanical system
              );

    std::string m_name;  ///< vehicle name
    ChSystem* m_system;  ///< pointer to the Chrono system
    bool m_ownsSystem;   ///< true if system created at construction

    std::shared_ptr<ChChassis> m_chassis;  ///< handle to the chassis subsystem

    double m_stepsize;  ///< integration step-size for the vehicle system
};

/// @} vehicle

}  // end namespace vehicle
}  // end namespace chrono

#endif
