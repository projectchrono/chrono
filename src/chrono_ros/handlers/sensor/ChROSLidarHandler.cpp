#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSLidarHandler::ChROSLidarHandler(std::shared_ptr<ChLidarSensor> camera) : ChROSHandler(), m_camera(camera) {}

bool ChROSLidarHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Check that the camera's filter list contains an access filter.
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<ChFilterDIAccess>(p) != nullptr;
    });
    if (it == m_filters.rend()) {
        GetLog() << "Lidar sensor must have a ChFilterDIAccess filter. Can't initialize handler.\n";
        return false;
    }

    auto node = interface->GetNode();
    m_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(ResolveROSName("point_cloud"), 1);

    return true;
}

void ChROSLidarHandler::Tick(double time) {
    sensor_msgs::msg::PointCloud2 pc;

    auto pc_ptr = m_camera->GetMostRecentBuffer<UserDIBufferPtr>();
    if (!pc_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Lidar buffer is not ready. Not ticking. \n";
        return;
    }

    // TODO
    GetLog() << "TODO: Lidar support is not provided yet. Publishing empty message.\n";
    // pc.header.time = GetROSTimestamp(time);
    // pc.width = pc_ptr->Width;
    // pc.height = pc_ptr->Height;
    // pc.size = sizeof(PixelDI);
    // pc.encoding = "pc";
    // pc.data = pc->Buffer.get();
    // pc.step = pc.size * pc.width;

    m_publisher->publish(pc);
}

}  // namespace ros
}  // namespace chrono