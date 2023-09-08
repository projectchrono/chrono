#ifndef CH_ROS_SENSOR_HANDLER_UTILITIES_H
#define CH_ROS_SENSOR_HANDLER_UTILITIES_H

#include "chrono_sensor/sensors/ChSensor.h"

#include <memory>

namespace chrono {
namespace ros {

class ChROSSensorHandlerUtilities {
  public:
    template <class FilterType, const char* FilterName>
    static bool CheckFilterList(std::shared_ptr<chrono::sensor::ChSensor> sensor) {
        auto filters = sensor->GetFilterList();
        auto it = std::find_if(filters.rbegin(), filters.rend(),
                               [](auto filter) { return std::dynamic_pointer_cast<FilterType>(filter) != nullptr; });
        if (it == filters.rend()) {
            GetLog() << "ERROR: Sensor with name '" << sensor->GetName().c_str() << "' must have a " << FilterName
                     << " filter. Can't initialize handler.\n";
            return false;
        }
        return true;
    }
};

}  // namespace ros
}  // namespace chrono

#endif