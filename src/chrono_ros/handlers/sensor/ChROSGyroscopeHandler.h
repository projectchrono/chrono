#ifndef CH_ROS_GYROSCOPE_HANDLER
#define CH_ROS_GYROSCOPE_HANDLER

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChIMUSensor.h"

namespace chrono {
namespace ros {

class ChROSGyroscopeHandler : public ChROSHandler {
  public:
    ChROSGyroscopeHandler(std::shared_ptr<chrono::sensor::ChGyroscopeSensor> imu);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChGyroscopeSensor> m_imu;

    sensor_msgs::msg::Imu m_imu_msg;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
