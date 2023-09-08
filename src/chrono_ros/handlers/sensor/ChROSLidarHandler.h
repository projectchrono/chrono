#ifndef CH_ROS_LIDAR_HANDLER
#define CH_ROS_LIDAR_HANDLER

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChLidarSensor.h"

namespace chrono {
namespace ros {

class ChROSLidarHandler : public ChROSHandler {
  public:
    ChROSLidarHandler(std::shared_ptr<chrono::sensor::ChLidarSensor> lidar);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChLidarSensor> m_lidar;

    sensor_msgs::msg::PointCloud2 m_lidar_msg;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
