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
// ChShaft-based manual transmission model constructed with data from file
// (JSON format).
//
// =============================================================================

#ifndef SHAFTS_MANUAL_TRANSMISSION_H
#define SHAFTS_MANUAL_TRANSMISSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/powertrain/ChManualTransmissionShafts.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_powertrain
/// @{

/// Shafts-based automatic transmission subsystem (specified through JSON file).
class CH_VEHICLE_API ManualTransmissionShafts : public ChManualTransmissionShafts {
  public:
    ManualTransmissionShafts(const std::string& filename);
    ManualTransmissionShafts(const rapidjson::Document& d);
    ~ManualTransmissionShafts() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;

    virtual double GetTransmissionBlockInertia() const override { return m_transmissionblock_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }
    virtual double GetMotorshaftInertia() const override { return m_motorshaft_inertia; }
    virtual double GetDriveshaftInertia() const override { return m_driveshaft_inertia; }
    virtual double GetClutchTorqueLimit() const override { return m_clutch_torque_limit; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    double m_motorshaft_inertia;
    double m_driveshaft_inertia;
    double m_transmissionblock_inertia;
    double m_ingear_shaft_inertia;

    double m_rev_gear;
    std::vector<double> m_fwd_gear;

    double m_clutch_torque_limit;
};

/// @} vehicle_powertrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
