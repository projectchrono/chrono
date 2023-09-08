#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"
#include "chrono_ros/handlers/sensor/ChROSSensorHandlerUtilities.h"

#include "chrono_sensor/filters/ChFilterAccess.h"

using namespace chrono::sensor;

namespace chrono {
namespace ros {

ChROSGPSHandler::ChROSGPSHandler(std::shared_ptr<ChGPSSensor> gps) : ChROSHandler(gps->GetUpdateRate()), m_gps(gps) {}

bool ChROSGPSHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    if (!ChROSSensorHandlerUtilities::CheckFilterList<ChFilterGPSAccess, ChFilterGPSAccessName>(m_gps)) {
        return false;
    }

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", m_gps->GetName(), "data");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = interface->GetNode()->create_publisher<sensor_msgs::msg::NavSatFix>(topic_name, 1);

    m_gps_msg.header.frame_id = m_gps->GetParent()->GetName();

    return true;
}

void ChROSGPSHandler::Tick(double time) {
    auto gps_ptr = m_gps->GetMostRecentBuffer<UserGPSBufferPtr>();
    if (!gps_ptr->Buffer) {
        // TODO: Is this supposed to happen?
        GetLog() << "GPS buffer is not ready. Not ticking. \n";
        return;
    }

    GPSData gps_data = gps_ptr->Buffer[0];
    m_gps_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(gps_data.Time);
    m_gps_msg.latitude = gps_data.Latitude;
    m_gps_msg.longitude = gps_data.Longitude;
    m_gps_msg.altitude = gps_data.Altitude;

    m_publisher->publish(m_gps_msg);
}

}  // namespace ros
}  // namespace chrono