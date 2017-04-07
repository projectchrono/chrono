// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor
// =============================================================================
//
// Simple powertrain model for the M113 vehicle.
// - both power and torque limited
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_SimpleMapPowertrain.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_SimpleMapPowertrain::Generic_SimpleMapPowertrain()
    : ChPowertrain(),
      m_motorSpeed(0),
      m_motorTorque(0),
      m_shaftTorque(0),
      m_gear_ratios({-0.3333, 0.2857, 0.5155, 0.7937, 1.0753, 1.3158, 1.4815}),
      m_zeroThrottleMap({0, 52.360, 104.720, 157.080, 209.440, 261.799, 314.159, 366.519, 418.879, 471.239, 523.599,
                         575.959, 628.319, 654.498, 680.678, 706.858, 733.038, 785.398, 837.758},
                        {0.000, - 39.262, -39.263, -39.266, -39.272, -39.284, -39.307, -39.353, -39.447, -39.625,
                         -39.750, -39.875, -40.250, -41.500, -43.000, -46.000, -52.000, -64.000, -800.000}),
      m_fullThrottleMap({0, 52.360, 104.720, 157.080, 209.440, 261.799, 314.159, 366.519, 418.879, 471.239, 523.599,
                         575.959, 628.319, 654.498, 680.678, 706.858, 733.038, 785.398, 837.758},
                        {80.000, 80.000, 135.000, 200.000, 245.000, 263.000, 310.000, 358.000, 404.000, 455.000,
                         475.000, 485.000, 468.000, 462.000, 455.000, 427.000, 370.000, 259.000, -700.000}) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_SimpleMapPowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    SetSelectedGear(1);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_SimpleMapPowertrain::SetSelectedGear(int igear) {
    assert(igear >= 0);
    assert(igear < m_gear_ratios.size());

    m_current_gear = igear;
    m_current_gear_ratio = m_gear_ratios[igear];
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_SimpleMapPowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
    m_drive_mode = mode;
    switch (mode) {
        case FORWARD:
            SetSelectedGear(1);
            break;
        case REVERSE:
            SetSelectedGear(0);
            break;
        case NEUTRAL:
            m_current_gear_ratio = 1e20;
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void Generic_SimpleMapPowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // The motorspeed is the shaft speed multiplied by gear ratio inversed: (limited to 8000rpm)
    m_motorSpeed = shaft_speed / m_current_gear_ratio;
    m_motorSpeed = m_motorSpeed > (8000. * CH_C_PI / 30.) ? (8000. * CH_C_PI / 30.) : m_motorSpeed;
    m_motorSpeed = m_motorSpeed < 0.0 ? 0.0 : m_motorSpeed;

    // Motor torque is linearly interpolated by throttle gas value:
    double zeroThrottleTorque;
    double fullThrottleTorque;
    double curve_dot;   // not used
    double curve_ddot;  // not used
    m_zeroThrottleMap.Evaluate(m_motorSpeed, zeroThrottleTorque, curve_dot, curve_ddot);
    m_fullThrottleMap.Evaluate(m_motorSpeed, fullThrottleTorque, curve_dot, curve_ddot);

    m_motorTorque = zeroThrottleTorque * (1 - throttle) + fullThrottleTorque * (throttle);

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
