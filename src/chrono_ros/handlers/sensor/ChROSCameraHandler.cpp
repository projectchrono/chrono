#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

#include <sstream>

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSCameraHandler::ChROSCameraHandler(std::shared_ptr<ChCameraSensor> camera)
    : ChROSHandler(camera->GetUpdateRate()), m_camera(camera) {}

bool ChROSCameraHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckFilterList<ChFilterRGBA8Access, ChFilterRGBA8AccessName>(m_camera)) {
        return false;
    }

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", m_camera->GetName(), "image");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::Image>(topic_name, 1);

    m_image.header.frame_id = m_camera->GetParent()->GetName();
    m_image.width = m_camera->GetWidth();
    m_image.height = m_camera->GetHeight();
    m_image.encoding = "rgba8";
    m_image.step = sizeof(PixelRGBA8) * m_image.width;
    m_image.data.resize(m_image.step * m_image.height);

    return true;
}

void ChROSCameraHandler::Tick(double time) {
    auto rgba8_ptr = m_camera->GetMostRecentBuffer<UserRGBA8BufferPtr>();
    if (!rgba8_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "Camera buffer is not ready. Not ticking. \n";
        return;
    }

    m_image.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(rgba8_ptr->Buffer.get());
    m_image.data.assign(ptr, ptr + m_image.step * m_image.height);

    // TODO: The buffer above may be released (?) after this call. Is this a problem? Guess is no.
    m_publisher->publish(m_image);
}

}  // namespace ros
}  // namespace chrono