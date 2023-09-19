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

#include "chrono_sensor/sensors/ChSensor.h"

#include <memory>

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{


/// Utility class with static functions that may be useful for sensor-specific ROS handlers
class ChROSSensorHandlerUtilities {
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
            GetLog() << "ERROR: Sensor with name '" << sensor->GetName().c_str() << "' doesn't have a " << FilterName
                     << " filter.\n";
            return false;
        }
        return true;
    }
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif