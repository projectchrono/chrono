#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSAccelerometerHandler::ChROSAccelerometerHandler(std::shared_ptr<ChAccelerometerSensor> imu)
    : ChROSHandler(imu->GetUpdateRate()), m_imu(imu) {}

bool ChROSAccelerometerHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckFilterList<ChFilterAccelAccess, ChFilterAccelAccessName>(m_imu)) {
        return false;
    }

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", m_imu->GetName(), "data");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Imu>(topic_name, 1);

    m_imu_msg.header.frame_id = m_imu->GetParent()->GetName();

    return true;
}

void ChROSAccelerometerHandler::Tick(double time) {
    auto imu_ptr = m_imu->GetMostRecentBuffer<UserAccelBufferPtr>();
    if (!imu_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Accelerometer buffer is not ready. Not ticking. \n";
        return;
    }

    AccelData imu_data = imu_ptr->Buffer[0];
    m_imu_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    m_imu_msg.linear_acceleration.x = imu_data.X;
    m_imu_msg.linear_acceleration.y = imu_data.Y;
    m_imu_msg.linear_acceleration.z = imu_data.Z;

    m_publisher->publish(m_imu_msg);
}

}  // namespace ros
}  // namespace chrono