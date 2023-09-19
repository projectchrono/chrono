#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSMagnetometerHandler::ChROSMagnetometerHandler(std::shared_ptr<ChMagnetometerSensor> imu)
    : ChROSHandler(imu->GetUpdateRate()), m_imu(imu) {}

bool ChROSMagnetometerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckFilterList<ChFilterMagnetAccess, ChFilterMagnetAccessName>(m_imu)) {
        return false;
    }

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", "magnetometer", m_imu->GetName(), "data");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::MagneticField>(topic_name, 1);

    m_mag_msg.header.frame_id = m_imu->GetParent()->GetName();

    return true;
}

void ChROSMagnetometerHandler::Tick(double time) {
    auto imu_ptr = m_imu->GetMostRecentBuffer<UserMagnetBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Magnetometer buffer is not ready. Not ticking. \n";
        return;
    }

    MagnetData imu_data = imu_ptr->Buffer[0];
    m_mag_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_mag_msg.magnetic_field.x = imu_data.X;
    m_mag_msg.magnetic_field.y = imu_data.Y;
    m_mag_msg.magnetic_field.z = imu_data.Z;

    m_publisher->publish(m_mag_msg);
}

}  // namespace ros
}  // namespace chrono