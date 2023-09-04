#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

#include "chrono_sensor/sensors/ChAccelerometerSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSAccelerometerHandler::ChROSAccelerometerHandler(std::shared_ptr<ChAccelerometerSensor> imu)
    : ChROSHandler(), m_imu(imu) {}

bool ChROSAccelerometerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Check that the camera's filter list contains an access filter.
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<ChFilterAccelAccess>(p) != nullptr;
    });
    if (it == m_filters.rend()) {
        GetLog() << "Accelerometer sensor must have a ChFilterAccelAccess filter. Can't initialize handler.\n";
        return false;
    }

    auto node = interface->GetNode();
    m_publisher = node->create_publisher<sensor_msgs::msg::Imu>(ResolveROSName("accelerometer"), 1);

    return true;
}

void ChROSAccelerometerHandler::Tick(double time) {
    sensor_msgs::msg::Imu msg;

    auto imu_ptr = m_imu->GetMostRecentBuffer<UserAccelBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Accelerometer buffer is not ready. Not ticking. \n";
        return;
    }

    AccelData imu_data = imu_ptr->Buffer[0];
    msg.header.time = GetROSTimestamp(time);
    msg.linear_acceleration.x = imu_data.X;
    msg.linear_acceleration.y = imu_data.Y;
    msg.linear_acceleration.z = imu_data.Z;

    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono