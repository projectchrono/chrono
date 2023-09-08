#ifndef CH_ROS_CAMERA_HANDLER
#define CH_ROS_CAMERA_HANDLER

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"

namespace chrono {
namespace ros {

class ChROSCameraHandler : public ChROSHandler {
  public:
    ChROSCameraHandler(std::shared_ptr<chrono::sensor::ChCameraSensor> camera);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChCameraSensor> m_camera;

    sensor_msgs::msg::Image m_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
