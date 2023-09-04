#ifndef CH_ROS_MAGNETOMETER_HANDLER
#define CH_ROS_MAGNETOMETER_HANDLER

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

namespace chrono {
namespace ros {

class ChROSMagnetometerHandler : public ChROSHandler {
  public:
    ChROSMagnetometerHandler(std::shared_ptr<chrono::sensor::ChMagnetometerSensor> imu);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChMagnetometerSensor> m_imu;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
