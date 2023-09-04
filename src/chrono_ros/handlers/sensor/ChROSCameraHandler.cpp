#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSCameraHandler::ChROSCameraHandler(std::shared_ptr<ChCameraSensor> camera) : ChROSHandler(), m_camera(camera) {}

bool ChROSCameraHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Check that the camera's filter list contains an access filter.
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<ChFilterRGBA8Access>(p) != nullptr;
    });
    if (it == m_filters.rend()) {
        GetLog() << "Camera sensor must have a ChFilterRGBA8Access filter. Can't initialize handler.\n";
        return false;
    }

    auto node = interface->GetNode();
    m_publisher = node->create_publisher<sensor_msgs::msg::Image>(ResolveROSName("image"), 1);

    return true;
}

void ChROSCameraHandler::Tick(double time) {
    sensor_msgs::msg::Image image;

    auto rgba8_ptr = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    if (!rgba8_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Camera buffer is not ready. Not ticking. \n";
        return;
    }

    image.header.time = GetROSTimestamp(time);
    image.width = rgba8_ptr->Width;
    image.height = rgba8_ptr->Height;
    image.size = sizeof(PixelRGBA8);
    image.encoding = "rgba8";
    image.data = rgba8->Buffer.get();
    image.step = image.size * image.width;

    m_publisher->publish(image);
}

}  // namespace ros
}  // namespace chrono