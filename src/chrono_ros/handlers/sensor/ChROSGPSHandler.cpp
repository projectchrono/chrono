#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_ros/utils/ChROSConversions.h"

#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGPSHandler::ChROSGPSHandler(std::shared_ptr<ChGPSSensor> gps) : ChROSHandler(), m_gps(gps) {}

bool ChROSGPSHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    // Check that the camera's filter list contains an access filter.
    auto it = std::find_if(m_filters.rbegin(), m_filters.rend(), [](std::shared_ptr<ChFilter> p) {
        return std::dynamic_pointer_cast<ChFilterGPSAccess>(p) != nullptr;
    });
    if (it == m_filters.rend()) {
        GetLog() << "GPS sensor must have a ChFilterGPSAccess filter. Can't initialize handler.\n";
        return false;
    }

    auto node = interface->GetNode();
    m_publisher = node->create_publisher<sensor_msgs::msg::NavSatFix>(ResolveROSName("gps"), 1);

    return true;
}

void ChROSGPSHandler::Tick(double time) {
    sensor_msgs::msg::NatSavFix msg;

    auto gps_ptr = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!gps_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "GPS buffer is not ready. Not ticking. \n";
        return;
    }

    GPSData gps_data = gps_ptr->Buffer[0];
    msg.header.time = GetROSTimestamp(gps_data.Time);
    msg.latitude = gps_data.Latitude;
    msg.longitude = gps_data.Longitude;
    msg.altitude = gps_data.Altitude;

    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono