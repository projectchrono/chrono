#ifndef CH_ROS_CLOCK_HANDLER
#define CH_ROS_CLOCK_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "rclcpp/publisher.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace chrono {
namespace ros {

class ChROSClockHandler : public ChROSHandler {
  public:
    ChROSClockHandler(uint64_t frequency = 0);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif