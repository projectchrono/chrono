// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Marcel Offermans
// =============================================================================
//
// Generic vehicle manual transmission model based on ChShaft objects.
//
// =============================================================================

#ifndef GENERIC_MANUAL_TRANSMISSION_SHAFTS_H
#define GENERIC_MANUAL_TRANSMISSION_SHAFTS_H

#include "chrono_vehicle/powertrain/ChManualTransmissionShafts.h"
#include "chrono_models/ChApiModels.h"

namespace chrono {
namespace vehicle {
namespace generic {

/// @addtogroup vehicle_models_generic
/// @{

class CH_MODELS_API Generic_ManualTransmissionShafts : public ChManualTransmissionShafts {
  public:
    Generic_ManualTransmissionShafts(const std::string& name);
    ~Generic_ManualTransmissionShafts() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    virtual double GetTransmissionBlockInertia() const override { return m_transmissionblock_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }
    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetClutchTorqueLimit() const override { return m_clutch_torque_limit; }

  private:
    // Shaft inertias.
    static const double m_transmissionblock_inertia;
    static const double m_motorshaft_inertia;
    static const double m_driveshaft_inertia;
    static const double m_ingear_shaft_inertia;

    // Clutch.
    static const double m_clutch_torque_limit;
};

/// @} vehicle_models_generic

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono

#endif  // GENERIC_MANUAL_TRANSMISSION_SHAFTS_H
