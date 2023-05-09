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
// Generic transmission model for a vehicle with no torque converter and a
// single forward gear.
//
// =============================================================================

#ifndef ACV_TRANSMISSION_SIMPLE_H
#define ACV_TRANSMISSION_SIMPLE_H

#include "chrono_vehicle/powertrain/ChAutomaticTransmissionSimpleMap.h"

class ACV_AutomaticTransmissionSimple : public chrono::vehicle::ChAutomaticTransmissionSimpleMap {
  public:
    ACV_AutomaticTransmissionSimple(const std::string& name);
    ~ACV_AutomaticTransmissionSimple() {}

    virtual void SetGearRatios(std::vector<double>& fwd, double& rev) override;
    virtual void SetShiftPoints(std::vector<std::pair<double, double>>& shift_bands) override;

  private:
    static const double m_fwd_gear_ratio;
    static const double m_rev_gear_ratio;
};

#endif
