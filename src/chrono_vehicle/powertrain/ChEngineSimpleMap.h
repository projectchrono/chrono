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
// Simple engine model based on torque-speed engine maps
//
// =============================================================================

#ifndef CH_SIMPLEMAP_ENGINE_H
#define CH_SIMPLEMAP_ENGINE_H

#include <utility>

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChEngine.h"

#include "chrono/motion_functions/ChFunction_Recorder.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Template for simple engine model based on speed-torque engine maps.
class CH_VEHICLE_API ChEngineSimpleMap : public ChEngine {
  public:
    virtual ~ChEngineSimpleMap() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "EngineSimpleMap"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motor_speed; }

    /// Return the output engine torque.
    /// This is the torque passed to a transmission subsystem.
    virtual double GetOutputMotorshaftTorque() const override { return m_motor_torque; }

  protected:
    ChEngineSimpleMap(const std::string& name);

    /// Specify maximum engine speed.
    virtual double GetMaxEngineSpeed() = 0;

    /// Set the engine speed-torque maps.
    /// A concrete class must add the speed-torque points to the provided maps, using the
    /// ChFunction_Recorder::AddPoint() function.
    virtual void SetEngineTorqueMaps(ChFunction_Recorder& map0,  ///< engine map at zero throttle
                                     ChFunction_Recorder& mapF   ///< engine map at full throttle
                                     ) = 0;

  private:
    /// Initialize the engine system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this engine system at the current time.
    /// The engine is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_speed             ///< input transmission speed
                             ) override;

    /// Advance the state of this engine system by the specified time step.
    /// This function does nothing for this simplified engine model.
    virtual void Advance(double step) override {}

    double m_motor_speed;   ///< current engine speed
    double m_motor_torque;  ///< current engine torque

    ChFunction_Recorder m_zero_throttle_map;  ///< engine map at zero throttle
    ChFunction_Recorder m_full_throttle_map;  ///< engine map at full throttle
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
