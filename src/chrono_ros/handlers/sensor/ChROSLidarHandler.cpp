#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSLidarHandler::ChROSLidarHandler(std::shared_ptr<ChLidarSensor> lidar)
    : ChROSHandler(lidar->GetUpdateRate()), m_lidar(lidar) {}

bool ChROSLidarHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckFilterList<ChFilterXYZIAccess, ChFilterXYZIAccessName>(m_lidar)) {
        return false;
    }

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", m_lidar->GetName(), "data");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 1);

    m_lidar_msg.header.frame_id = "map";  // m_lidar->GetParent()->GetName();
    m_lidar_msg.width = m_lidar->GetWidth();
    m_lidar_msg.height = m_lidar->GetHeight();
    m_lidar_msg.is_bigendian = false;
    m_lidar_msg.is_dense = true;
    m_lidar_msg.row_step = sizeof(PixelXYZI) * m_lidar_msg.width;
    m_lidar_msg.point_step = sizeof(PixelXYZI);
    m_lidar_msg.data.resize(m_lidar_msg.row_step * m_lidar_msg.height);

    m_lidar_msg.fields.resize(4);
    const std::string field_names[4] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; i++) {
        m_lidar_msg.fields[i].name = field_names[i];
        m_lidar_msg.fields[i].offset = sizeof(float) * i;
        m_lidar_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        m_lidar_msg.fields[i].count = 1;
    }

    return true;
}

void ChROSLidarHandler::Tick(double time) {
    auto pc_ptr = m_lidar->GetMostRecentBuffer<UserXYZIBufferPtr>();
    if (!pc_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Lidar buffer is not ready. Not ticking. \n";
        return;
    }

    // TODO
    m_lidar_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(pc_ptr->Buffer.get());
    m_lidar_msg.data.assign(ptr, ptr + m_lidar_msg.row_step * m_lidar_msg.height);

    m_publisher->publish(m_lidar_msg);
}

}  // namespace ros
}  // namespace chrono