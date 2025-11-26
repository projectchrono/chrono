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
// Authors: Aaron Young
// =============================================================================
//
// Utilities useful for sensor-specific ROS handlers
//
// =============================================================================

#ifndef CH_ROS_SENSOR_HANDLER_UTILITIES_H
#define CH_ROS_SENSOR_HANDLER_UTILITIES_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_sensor/sensors/ChSensor.h"

#include <memory>
#include <array>

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// Utility class with static functions that may be useful for sensor-specific ROS handlers
class CH_ROS_API ChROSSensorHandlerUtilities {
  public:
    /// Check for filter in Sensor filter list. Returns true if present, false if not.
    /// @tparam FilterType the filter to search for
    /// @tparam FilterName name of the filter to search for
    template <class FilterType, const char* FilterName>
    static bool CheckSensorHasFilter(std::shared_ptr<chrono::sensor::ChSensor> sensor) {
        auto filters = sensor->GetFilterList();
        auto it = std::find_if(filters.rbegin(), filters.rend(),
                               [](auto filter) { return std::dynamic_pointer_cast<FilterType>(filter) != nullptr; });
        if (it == filters.rend()) {
            std::cerr << "ERROR: Sensor with name '" << sensor->GetName().c_str() << "' doesn't have a " << FilterName
                      << " filter." << std::endl;
            return false;
        }
        return true;
    }

    /// Calculates the covariance of the sensor data
    /// @param data the sensor data
    /// @return the covariance of the sensor data
    template <typename T = double, unsigned long N = 3>
    static std::array<T, N * N> CalculateCovariance(const std::array<T, N>& data,
                                                    const std::array<T, N>& mean,
                                                    unsigned long count) {
        std::array<T, N * N> covariance;
        std::fill(covariance.begin(), covariance.end(), T(0));
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
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