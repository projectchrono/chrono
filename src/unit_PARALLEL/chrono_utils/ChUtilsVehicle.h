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
// Utility classes for using Chrono::Vehicle models in a ChronoParallel
// application.
//
// =============================================================================

#ifndef CH_UTILS_VEHICLE_H
#define CH_UTILS_VEHICLE_H

#include <string>
#include <iostream>
#include <sstream>
#include <fstream>

#include "chrono_utils/ChApiUtils.h"

#include "subsys/ChVehicleModelData.h"
#include "subsys/vehicle/Vehicle.h"
#include "subsys/powertrain/SimplePowertrain.h"

namespace chrono {
namespace utils {

// -----------------------------------------------------------------------------
/// Callback class for specifying driver inputs at a specified time.
class CH_UTILS_API DriverInputsCallback {
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
class CH_UTILS_API TireContactCallback {
 public:
  /// A derived class must implement the function onCallback() which must append
  /// the contact shape(s) to the specified wheel body.
  virtual void onCallback(ChSharedPtr<ChBody> wheelBody,  ///< Pointer to the wheel body
                          double radius,                  ///< wheel radius
                          double width                    ///< wheel width
                          ) = 0;
};

// -----------------------------------------------------------------------------
/// Wrapper class for a Chrono::Vehicle vehicle and powertrain assembly.
/// A VehicleSystem always uses rigid tires (attached through a user-provided
/// callback object of type TireContactCallback) and uses driver inputs provided
/// through a user-provided callback object of type DriverInputsCallabck.
class CH_UTILS_API VehicleSystem {
 public:
  /// Constructor from given JSON specification files.
  /// The vehicle system is constructed within the specified system, using the
  /// given JSON specification files for the vehicle and powertrain subsystems.
  /// These files are assumed to be given relative to the Chrono::Vehicle data
  /// directory.
  VehicleSystem(ChSystem*          system,                  ///< pointer to containing system
                const std::string& vehicle_def_filename,    ///< JSON file with vehicle specification
                const std::string& powertrain_def_filename  ///< JSON file with powertrain specification
                );

  /// Set the callback object for specifying driver inputs.
  /// If not set, all driver inputs default to 0.
  void SetDriverInputsCallback(DriverInputsCallback* cb) { m_driver_cb = cb; }

  /// Set the callback object for specifying tire contact geometry.
  /// Note that this function must be called before Initialize().
  /// If not set, all tire contact geometry defaults to cylinders.
  void SetTireContactCallback(TireContactCallback* cb) { m_tire_cb = cb; }

  /// Initialize the vehicle model at the specified location and orientation.
  void Initialize(const ChVector<>& init_loc,     ///< initial location of the chassis reference frame
                  const ChQuaternion<>& init_rot  ///< initial orientation of the chassis reference frame
                  );

  /// Update the vehicle model at the specified time.
  void Update(double time);

 private:
  ChSharedPtr<Vehicle> m_vehicle;
  ChSharedPtr<SimplePowertrain> m_powertrain;
  ChTireForces m_tire_forces;
  DriverInputsCallback* m_driver_cb;
  TireContactCallback* m_tire_cb;
};

}  // namespace utils
}  // namespace chrono

#endif
