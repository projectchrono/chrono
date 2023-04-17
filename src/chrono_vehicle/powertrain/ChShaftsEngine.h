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

#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsThermalEngine.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

// Forward reference
class ChVehicle;

/// Template for an engine model using shaft elements.
class CH_VEHICLE_API ChShaftsEngine : public ChEngine {
  public:
    virtual ~ChShaftsEngine();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ShaftsEngine"; }

    /// Return the current engine speed.
    virtual double GetMotorSpeed() const override { return m_motorshaft->GetPos_dt(); }

    /// Return the current engine torque.
    virtual double GetMotorTorque() const override { return m_engine->GetTorqueReactionOn1(); }

  protected:
    /// Construct a shafts-based engine model.
    ChShaftsEngine(const std::string& name, const ChVector<>& dir_motor_block = ChVector<>(1, 0, 0));

    /// Set inertia of the motor block.
    virtual double GetMotorBlockInertia() const = 0;

    /// Set inertia of the motorshaft (crankshaft + fly wheel).
    virtual double GetCrankshaftInertia() const = 0;

    /// Engine speed-torque map.
    virtual void SetEngineTorqueMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

    /// Engine speed-torque braking effect because of losses.
    virtual void SetEngineLossesMap(std::shared_ptr<ChFunction_Recorder>& map) = 0;

  private:
    /// Initialize this engine system.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Update the state of this engine system at the current time.
    /// The engine system is provided the current driver throttle input, a value in the range [0,1].
    virtual void Synchronize(double time,                        ///< [in] current time
                             const DriverInputs& driver_inputs,  ///< [in] current driver inputs
                             double shaft_speed                  ///< [in] motorshaft speed
                             ) override;

    /// Advance the state of this engine system by the specified time step.
    /// Since the state of a ShaftsEngine is advanced as part of the vehicle state, this function does nothing.
    virtual void Advance(double step) override {}

    std::shared_ptr<ChShaft> m_motorblock;
    std::shared_ptr<ChShaftsBody> m_motorblock_to_body;
    std::shared_ptr<ChShaftsThermalEngine> m_engine;
    std::shared_ptr<ChShaftsThermalEngine> m_engine_losses;

    ChVector<> m_dir_motor_block;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
