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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Manual transmission model template based on ChShaft objects consisting of
// a clutch and a manual gearbox.
//
// =============================================================================

#ifndef CH_SHAFTS_MANUAL_TRANSMISSION_H
#define CH_SHAFTS_MANUAL_TRANSMISSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTransmission.h"

#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGearbox.h"
#include "chrono/physics/ChShaftsBody.h"
#include "chrono/physics/ChShaftsClutch.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

// Forward reference
class ChVehicle;

/// Template for a manual transmission model using shaft elements.
/// This transmission template includes a clutch and a manual gearbox.
class CH_VEHICLE_API ChManualTransmissionShafts : public ChManualTransmission {
  public:
    /// Construct a shafts-based manual transmission model.
    ChManualTransmissionShafts(const std::string& name);

    virtual ~ChManualTransmissionShafts();

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "ManualTransmissionShafts"; }

    /// Return true if a clutch model is included.
    /// A ChManualTransmissionShafts includes a clutch model.
    virtual bool HasClutch() const override { return true; }

    /// Return the transmission output torque on the driveshaft.
    /// This is the torque that is passed to the driveline subsystem, thus providing the interface between the
    /// powertrain and vehicle systems.
    virtual double GetOutputDriveshaftTorque() const override;

    /// Return the transmission output speed of the motorshaft.
    /// This represents the output from the transmision subsystem that is passed to the engine subsystem.
    virtual double GetOutputMotorshaftSpeed() const override;

  protected:
    /// Set inertia of the transmission block.
    virtual double GetTransmissionBlockInertia() const = 0;

    /// Inertias of the component ChShaft objects.
    virtual double GetIngearShaftInertia() const = 0;

    /// Inertia of the motorshaft (connection to engine).
    virtual double GetMotorshaftInertia() const = 0;

    /// Inertia of the driveshaft (connection to driveline).
    virtual double GetDriveshaftInertia() const = 0;

    /// Maximum torque that the clutch can transmit without slipping.
    virtual double GetClutchTorqueLimit() const = 0;

  private:
    /// Initialize this transmission system by attaching it to an existing vehicle chassis and connecting the provided
    /// engine and driveline subsystems.
    virtual void Initialize(std::shared_ptr<ChChassis> chassis) override;

    /// Synchronize the state of this transmission system at the current time.
    virtual void Synchronize(double time,                        ///< current time
                             const DriverInputs& driver_inputs,  ///< current driver inputs
                             double motorshaft_torque,           ///< input engine torque
                             double driveshaft_speed             ///< input driveline speed
                             ) override;

    /// Advance the state of this transmission system by the specified time step.
    /// No-op, since the state of a AutomaticTransmissionShafts is advanced as part of the vehicle state.
    virtual void Advance(double step) override {}

    /// Perform any action required on a gear shift (the new gear and gear ratio are available).
    virtual void OnGearShift() override;

    /// Perform any action required on placing the transmission in neutral.
    virtual void OnNeutralShift() override;

    std::shared_ptr<ChShaft> m_motorshaft;  ///< shaft connection to the transmission
    std::shared_ptr<ChShaft> m_driveshaft;  ///< shaft connection to driveline

    std::shared_ptr<ChShaft> m_transmissionblock;
    std::shared_ptr<ChShaftsBody> m_transmissionblock_to_body;
    std::shared_ptr<ChShaftsGearbox> m_gears;

    // Extras for clutch
    std::shared_ptr<ChShaftsClutch> m_clutch;
    std::shared_ptr<ChShaft> m_clutchShaft;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
