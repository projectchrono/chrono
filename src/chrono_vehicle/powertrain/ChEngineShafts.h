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
// Engine model template based on ChShaft objects.
//
// =============================================================================

#ifndef CH_SHAFTS_ENGINE_H
#define CH_SHAFTS_ENGINE_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChEngine.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsThermalEngine.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

// Forward reference
class ChVehicle;

/// Template for an engine model using shaft elements.
class CH_VEHICLE_API ChEngineShafts : public ChEngine {
  public:
    virtual ~ChEngineShafts();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "EngineShafts"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motorshaft->GetPos_dt(); }

    /// Return the output engine torque.
    /// This is the torque passed to a transmission subsystem.
    virtual double GetOutputMotorshaftTorque() const override { return m_engine->GetTorqueReactionOn1(); }
    //// TODO: is this correct?  shouldn't we also add the reaction torque from engine_losses?!?

  protected:
    /// Construct a shafts-based engine model.
    ChEngineShafts(const std::string& name, const ChVector<>& dir_motor_block = ChVector<>(1, 0, 0));

    /// Set inertia of the motor block.
    virtual double GetMotorBlockInertia() const = 0;

    /// Set inertia of the motorshaft (crankshaft + fly wheel).
    virtual double GetMotorshaftInertia() const = 0;

    /// Engine speed-torque map.
    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

    /// Engine speed-torque braking effect because of losses.
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

  private:
    /// Initialize this engine system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this engine system at the current time.
    /// Set the motorshaft speed to the provided value and update the throttle level for the engine.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_speed             ///< input transmission speed
                             ) override;

    /// Advance the state of this engine system by the specified time step.
    /// Since the state of a EngineShafts is advanced as part of the vehicle state, this function does nothing.
    virtual void Advance(double step) override {}

    std::shared_ptr<ChShaft> m_motorblock;
    std::shared_ptr<ChShaftsBody> m_motorblock_to_body;
    std::shared_ptr<ChShaftsThermalEngine> m_engine;
    std::shared_ptr<ChShaftsThermalEngine> m_engine_losses;
    std::shared_ptr<ChShaft> m_motorshaft;  ///< shaft connection to the transmission

    ChVector<> m_dir_motor_block;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
