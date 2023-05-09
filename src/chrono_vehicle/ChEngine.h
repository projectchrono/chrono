// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Base class for a vehicle engine.
//
// =============================================================================

#ifndef CH_ENGINE_H
#define CH_ENGINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/ChPart.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Base class for an engine subsystem.
class CH_VEHICLE_API ChEngine : public ChPart {
  public:
    virtual ~ChEngine() {}

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const = 0;

    /// Return the output engine torque on the motorshaft.
    /// This is the torque passed to a transmission subsystem.
    virtual double GetOutputMotorshaftTorque() const = 0;

  protected:
    ChEngine(const std::string& name = "");

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Initialize this engine.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis);

    /// Update the engine system at the current time.
    /// The engine is provided the current driver throttle input, a value in the range [0,1].
    /// The motorshaft speed represents the input to the engine from the transmision system.
    /// This default implementation sets the speed of the motorshaft.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_speed             ///< input transmission speed
                             ) = 0;

    /// Advance the state of the engine by the specified time step.
    virtual void Advance(double step) {}

    friend class ChPowertrainAssembly;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
