#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

#include "chrono_sensor/sensors/ChGyroscopeSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGyroscopeHandler::ChROSGyroscopeHandler(std::shared_ptr<ChGyroscopeSensor> imu) : ChROSHandler(), m_imu(imu) {}

bool ChROSGyroscopeHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Check that the camera's filter list contains an access filter.
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<ChFilterGyroAccess>(p) != nullptr;
    });
    if (it == m_filters.rend()) {
        GetLog() << "Gyroscope sensor must have a ChFilterGyroAccess filter. Can't initialize handler.\n";
        return false;
    }

    auto node = interface->GetNode();
    m_publisher = node->create_publisher<sensor_msgs::msg::Imu>(ResolveROSName("gyroscope"), 1);

    return true;
}

void ChROSGyroscopeHandler::Tick(double time) {
    sensor_msgs::msg::Imu msg;

    auto imu_ptr = m_imu->GetMostRecentBuffer<UserGyroBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Gyroscope buffer is not ready. Not ticking. \n";
        return;
    }

    GyroData imu_ptr->Buffer[0];
    msg.header.time = GetROSTimestamp(time);
    msg.angular_velocity.x = imu_data.Roll;
    msg.angular_velocity.y = imu_data.Pitch;
    msg.angular_velocity.z = imu_data.Yaw;

    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono