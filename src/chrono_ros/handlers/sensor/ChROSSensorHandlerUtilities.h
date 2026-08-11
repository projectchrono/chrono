// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// Utilities shared by the sensor-specific ROS handlers.
//
// =============================================================================

#ifndef CH_ROS_SENSOR_HANDLER_UTILITIES_H
#define CH_ROS_SENSOR_HANDLER_UTILITIES_H

#include "chrono_sensor/sensors/ChSensor.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <memory>

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// Static helpers shared by the sensor handlers.
class ChROSSensorHandlerUtilities {
  public:
    /// Returns true if 'sensor' has a filter of the given type in its filter list (the
    /// access filter that exposes a host-side buffer). Logs and returns false otherwise.
    template <class FilterType, const char* FilterName>
    static bool CheckSensorHasFilter(std::shared_ptr<chrono::sensor::ChSensor> sensor) {
        auto filters = sensor->GetFilterList();
        auto it = std::find_if(filters.rbegin(), filters.rend(),
                               [](auto filter) { return std::dynamic_pointer_cast<FilterType>(filter) != nullptr; });
        if (it == filters.rend()) {
            std::cerr << "ERROR: Sensor '" << sensor->GetName() << "' is missing a " << FilterName
                      << " filter; the corresponding ROS handler cannot read its data." << std::endl;
            return false;
        }
        return true;
    }

    /// Rolling pseudo-covariance of a sample about its running mean (Chrono sensors do not
    /// emit covariance, so handlers approximate it).
    template <typename T = double, unsigned long N = 3>
    static std::array<T, N * N> CalculateCovariance(const std::array<T, N>& data,
                                                    const std::array<T, N>& mean,
                                                    unsigned long count) {
        std::array<T, N * N> covariance;
        std::fill(covariance.begin(), covariance.end(), T(0));
        for (unsigned long i = 0; i < N; i++) {
            for (unsigned long j = 0; j < N; j++) {
                covariance[i * N + j] = (data[i] - mean[i]) * (data[j] - mean[j]) / count;
            }
        }
        return covariance;
    }
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
