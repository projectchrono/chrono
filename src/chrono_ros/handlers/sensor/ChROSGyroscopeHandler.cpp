#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGyroscopeHandler::ChROSGyroscopeHandler(std::shared_ptr<ChGyroscopeSensor> imu)
    : ChROSHandler(imu->GetUpdateRate()), m_imu(imu) {}

bool ChROSGyroscopeHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckFilterList<ChFilterGyroAccess, ChFilterGyroAccessName>(m_imu)) {
        return false;
    }

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", "gyroscope", m_imu->GetName(), "data");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Imu>(topic_name, 1);

    m_imu_msg.header.frame_id = m_imu->GetParent()->GetName();

    return true;
}

void ChROSGyroscopeHandler::Tick(double time) {
    auto imu_ptr = m_imu->GetMostRecentBuffer<UserGyroBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Gyroscope buffer is not ready. Not ticking. \n";
        return;
    }

    GyroData imu_data = imu_ptr->Buffer[0];
    m_imu_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_imu_msg.angular_velocity.x = imu_data.Roll;
    m_imu_msg.angular_velocity.y = imu_data.Pitch;
    m_imu_msg.angular_velocity.z = imu_data.Yaw;

    m_publisher->publish(m_imu_msg);
}

}  // namespace ros
}  // namespace chrono