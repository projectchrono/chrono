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
// Utility classes for wrapping a wheeled vehicle and powertrain system into a
// single assembly.
//
// =============================================================================

#ifndef CH_WHEELED_VEHICLE_ASSEMBLY_H
#define CH_WHEELED_VEHICLE_ASSEMBLY_H

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono_vehicle/ChApiVehicle.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/powertrain/SimplePowertrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_utils
/// @{

// -----------------------------------------------------------------------------
/// Callback class for specifying driver inputs at a specified time.
class CH_VEHICLE_API ChDriverInputsCallback {
  public:
    /// A derived class must implement the function onCallback() which must
    /// return the throttle, steering, and braking driver inputs at the
    /// specified time.
    virtual void onCallback(double time,       ///< current time
                            double& throttle,  ///< [output] throttle driver input
                            double& steering,  ///< [output] steering driver input
                            double& braking    ///< [output] braking driver input
                            ) = 0;
};

// -----------------------------------------------------------------------------
/// Callback class for specifying tire contact geometry.
class CH_VEHICLE_API ChTireContactCallback {
  public:
    /// Callback function for attaching contact to a wheel body.
    /// A derived class must implement the function onCallback() which must
    /// append the contact shape(s) to the specified wheel body. Optionally,
    /// this function can first change the collision model type for the
    /// provided wheel body (consistent with the system type).
    virtual void onCallback(std::shared_ptr<ChBody> wheelBody  ///< Pointer to the wheel body
                            ) = 0;
};

// -----------------------------------------------------------------------------
/// Callback class for specifying chassis contact geometry.
class CH_VEHICLE_API ChChassisContactCallback {
  public:
    /// Callback function for attaching contact to the vehicle chassis body.
    /// A derived class must implement the function onCallback() which must
    /// append the contact shape(s) to the provided chassis body. Optionally,
    /// this function can first change the collision model type for the
    /// provided chassis body (consistent with the system type).
    virtual void onCallback(std::shared_ptr<ChBodyAuxRef> chassisBody  ///< Pointer to the chassis body
                            ) = 0;
};

// -----------------------------------------------------------------------------
/// Wrapper class for a Chrono::Vehicle vehicle and powertrain assembly.
/// A ChWheeledVehicleAssembly always uses rigid tires (attached through a
/// user-provided callback object of type ChTireContactCallback) and optionally
/// a contact model associated with the chassis body (specified through a
/// user-provided callback of type ChChassisContactCallback). Driver inputs are
/// provided through a user-provided callback object of type ChDriverInputsCallabck.
class CH_VEHICLE_API ChWheeledVehicleAssembly {
  public:
    /// Constructor from given JSON specification files.
    /// The vehicle system is constructed within the specified system, using the
    /// given JSON specification files for the vehicle and powertrain subsystems.
    /// These files are assumed to be given relative to the Chrono::Vehicle data
    /// directory.
    ChWheeledVehicleAssembly(ChSystem* system,                           ///< pointer to containing system
                             const std::string& vehicle_def_filename,    ///< JSON file with vehicle specification
                             const std::string& powertrain_def_filename  ///< JSON file with powertrain specification
                             );

    /// Set the callback object for specifying driver inputs.
    /// If not set, all driver inputs default to 0.
    void SetDriverInputsCallback(ChDriverInputsCallback* cb) { m_driver_cb = cb; }

    /// Set the callback object for specifying tire contact geometry.
    /// Note that this function must be called before Initialize().
    /// If not set, no contact geometry is attached to the vehicle wheels.
    void SetTireContactCallback(ChTireContactCallback* cb) { m_tire_cb = cb; }

    /// Set the callback object for specifying chassis contact geometry.
    /// Note that this function must be called before Initialize().
    /// If not set, no contact geometry is attached to the vehicle chassis.
    void SetChassisContactCallback(ChChassisContactCallback* cb) { m_chassis_cb = cb; }

    /// Initialize the vehicle model at the specified location and orientation.
    void Initialize(const ChVector<>& init_loc,     ///< initial location of the chassis reference frame
                    const ChQuaternion<>& init_rot  ///< initial orientation of the chassis reference frame
                    );

    /// Update the vehicle model at the specified time.
    void Synchronize(double time);

    /// Get handle to the underlying vehicle subsystem.
    std::shared_ptr<WheeledVehicle> GetVehicle() const { return m_vehicle; }

    /// Get handle to the underlying powertrain subsystem.
    std::shared_ptr<SimplePowertrain> GetPowertrain() const { return m_powertrain; }

  private:
    std::shared_ptr<WheeledVehicle> m_vehicle;
    std::shared_ptr<SimplePowertrain> m_powertrain;
    TireForces m_tire_forces;
    ChDriverInputsCallback* m_driver_cb;
    ChTireContactCallback* m_tire_cb;
    ChChassisContactCallback* m_chassis_cb;
};

/// @} vehicle_wheeled_utils

}  // end namespace vehicle
}  // namespace chrono

#endif
