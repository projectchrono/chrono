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

#include "chrono/physics/ChShaft.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChPart.h"
#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Base class for an engine subsystem.
class CH_VEHICLE_API ChEngine : public ChPart {
  public:
    virtual ~ChEngine();

    /// Get a handle to the underlying motorshaft (connection to a transmission).
    std::shared_ptr<ChShaft> GetMotorshaft() const { return m_motorshaft; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const = 0;

    /// Return the current engine torque.
    /// This is the torque passed to a transmission subsystem.
    virtual double GetMotorTorque() const = 0;

  protected:
    ChEngine(const std::string& name);

    virtual void InitializeInertiaProperties() override;
    virtual void UpdateInertiaProperties() override;

    /// Initialize this engine system by attaching it to an existing vehicle chassis.
    /// A derived class override must first call this base class version.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis);

    /// Synchronize the state of the engine at the current time.
    /// The engine is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double shaft_speed                  ///< [in] motorshaft speed
                             ) = 0;

    /// Advance the state of the engine by the specified time step.
    virtual void Advance(double step) {}

    std::shared_ptr<ChShaft> m_motorshaft;  ///< shaft connection to the transmission

  private:
    friend class ChWheeledVehicle;
    friend class ChTrackedVehicle;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
