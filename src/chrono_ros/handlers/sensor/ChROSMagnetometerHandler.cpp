#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

#include "chrono_sensor/sensors/ChMagnetometerSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSMagnetometerHandler::ChROSMagnetometerHandler(std::shared_ptr<ChMagnetometerSensor> imu)
    : ChROSHandler(), m_imu(imu) {}

bool ChROSMagnetometerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Check that the camera's filter list contains an access filter.
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<ChFilterMagnetAccess>(p) != nullptr;
    });
    if (it == m_filters.rend()) {
        GetLog() << "Magnetometer sensor must have a ChFilterMagnetAccess filter. Can't initialize handler.\n";
        return false;
    }

    auto node = interface->GetNode();
    m_publisher = node->create_publisher<sensor_msgs::msg::Imu>(ResolveROSName("magnetometer"), 1);

    return true;
}

void ChROSMagnetometerHandler::Tick(double time) {
    sensor_msgs::msg::Imu msg;

    auto imu_ptr = m_imu->GetMostRecentBuffer<UserMagnetBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Magnetometer buffer is not ready. Not ticking. \n";
        return;
    }

    MagnetData imu_data = imu_ptr->Buffer[0];
    msg.header.time = GetROSTimestamp(time);
    msg.magnetic_field.x = imu_data.X;
    msg.magnetic_field.y = imu_data.Y;
    msg.magnetic_field.z = imu_data.Z;

    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono