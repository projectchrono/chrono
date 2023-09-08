#ifndef CH_ROS_GPS_HANDLER
#define CH_ROS_GPS_HANDLER

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChGPSSensor.h"

namespace chrono {
namespace ros {

class ChROSGPSHandler : public ChROSHandler {
  public:
    ChROSGPSHandler(std::shared_ptr<chrono::sensor::ChGPSSensor> gps);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChGPSSensor> m_gps;

    sensor_msgs::msg::NavSatFix m_gps_msg;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
