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
// Authors: Radu Serban, Rainer Gericke
// =============================================================================
//
// Automatic transmission model template to represent a CVT (Continuous Variable
// Transmission).
//
// =============================================================================

#include "ChAutomaticTransmissionSimpleCVT.h"

namespace chrono {
namespace vehicle {

ChAutomaticTransmissionSimpleCVT::ChAutomaticTransmissionSimpleCVT(const std::string& name)
    : ChAutomaticTransmission(name), m_motorshaft_speed(0), m_driveshaft_torque(0) {}

void ChAutomaticTransmissionSimpleCVT::Initialize(std::shared_ptr<ChChassis> chassis) {
    ChTransmission::Initialize(chassis);
}

void ChAutomaticTransmissionSimpleCVT::SetOperationRange(double driveshaft_speed_start,
                                                         double ratio_start,
                                                         double driveshaft_speed_end,
                                                         double ratio_end,
                                                         double gearbox_effiency) {
    if (driveshaft_speed_start <= 0.0) {
        std::cout << "Error in " << __func__ << "(): driveshaft_speed_start must be > 0.0!" << std::endl;
        std::exit(11);
    }
    if (driveshaft_speed_end < 10.0 * driveshaft_speed_start) {
        std::cout << "Error in " << __func__ << "(): driveshaft_speed_start must be > 10.0 * driveshaft_speed_end!"
                  << std::endl;
        std::exit(12);
    }
    if (ratio_start <= 0.0 || ratio_end <= 0.0) {
        std::cout << "Error in " << __func__ << "(): ratio_start and ratio_end must be > 0.0!" << std::endl;
        std::exit(13);
    }
    if (ratio_start > ratio_end) {
        std::cout << "Error in " << __func__ << "(): ratio_start must be < ratio_end!" << std::endl;
        std::exit(14);
    }
    if (gearbox_effiency < 0.8 || gearbox_effiency > 1.0) {
        std::cout << "Error in " << __func__ << "(): gearbox_efficiency must be >= 0.8 and < 1.0!" << std::endl;
        std::exit(15);
    }

    m_efficiency = gearbox_effiency;

    // setup function object with linear interpolation and extrapolation
    m_cvt_gear_ratios.AddPoint(-100.0, ratio_start);
    m_cvt_gear_ratios.AddPoint(driveshaft_speed_start, ratio_start);
    m_cvt_gear_ratios.AddPoint(driveshaft_speed_end, ratio_end);
    m_cvt_gear_ratios.AddPoint(driveshaft_speed_end + 100.0, ratio_end);
    m_cvt_gear_ratios.SetExtrapolate(true);
}

void ChAutomaticTransmissionSimpleCVT::Synchronize(double time,
                                                   const DriverInputs& driver_inputs,
                                                   double motorshaft_torque,
                                                   double driveshaft_speed) {
    // Automatic gear ratio calculation for current driveshaft speed
    m_current_gear_ratio = m_cvt_gear_ratios.GetVal(std::abs(driveshaft_speed));

    // Set speed of the motorshaft (transmission output to the engine)
    m_motorshaft_speed = std::abs(driveshaft_speed) / m_current_gear_ratio;

    double sgn = 0.0;
    switch (GetDriveMode()) {
        case DriveMode::FORWARD:
            sgn = 1.0;
            break;
        case DriveMode::REVERSE:
            sgn = -1.0;
            break;
        case DriveMode::NEUTRAL:
            sgn = 0;
            break;
    }
    // Set torque at driveshaft (transmission output to the driveline)
    m_driveshaft_torque = sgn * m_efficiency * motorshaft_torque / m_current_gear_ratio;
}

void ChAutomaticTransmissionSimpleCVT::SetGearRatios(std::vector<double>& fwd, double& rev) {
    // we must set dummy values for these unused values to pass the assertions in debug mode
    rev = -1.0;
    fwd.push_back(1.0);
}

// -----------------------------------------------------------------------------

void ChAutomaticTransmissionSimpleCVT::PopulateComponentList() {}

}  // namespace vehicle
}  // namespace chrono