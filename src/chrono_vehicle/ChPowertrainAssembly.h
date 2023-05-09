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
// Definition of a vehicle powertrain assembly.
// This is an aggregate of an engine and a transmission.
//
// =============================================================================

#ifndef CH_POWERTRAIN_ASSEMBLY_H
#define CH_POWERTRAIN_ASSEMBLY_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChEngine.h"
#include "chrono_vehicle/ChTransmission.h"
#include "chrono_vehicle/ChDriveline.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Defintion of a powertrain assembly system.
/// A powertrain assembly consists of an engine and a transmission.
class CH_VEHICLE_API ChPowertrainAssembly {
  public:
    /// Construct a powertrain assembly with the specified engine and transmission subsystems.
    ChPowertrainAssembly(std::shared_ptr<ChEngine> engine, std::shared_ptr<ChTransmission> transmission)
        : m_engine(engine), m_transmission(transmission) {}

    ~ChPowertrainAssembly() {}

    const std::shared_ptr<ChEngine> GetEngine() const { return m_engine; }
    const std::shared_ptr<ChTransmission> GetTransmission() const { return m_transmission; }

    /// Return the transmission output torque on the driveshaft.
    /// This is the torque that is passed to the driveline subsystem, thus providing the interface between the
    /// powertrain and vehicle systems.
    double GetOutputTorque() const { return m_transmission->GetOutputDriveshaftTorque(); }

    /// Initialize this powertrain system by attaching it to an existing vehicle chassis.
    void Initialize(std::shared_ptr<ChChassis> chassis);

    /// Synchronize the state of this powertrain system at the current time.
    void Synchronize(double time,                        ///< current time
                     const DriverInputs& driver_inputs,  ///< current driver inputs
                     double driveshaft_speed             ///< input driveline speed
    );

    /// Advance the state of this powertrain system by the specified time step.
    void Advance(double step);

  private:
    std::shared_ptr<ChEngine> m_engine;
    std::shared_ptr<ChTransmission> m_transmission;

    friend class ChWheeledVehicle;
    friend class ChTrackedVehicle;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
