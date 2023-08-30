#ifndef CH_ROS_CLOCK_HANDLER
#define CH_ROS_CLOCK_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChSystem.h"

#include "rclcpp/publisher.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace chrono {
namespace ros {
class ChROSClockHandler : public ChROSHandler {
  public:
    ChROSClockHandler(ChSystem* system);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    virtual const std::string GetNamespace() const override { return "ChROSClockHandler"; }

  protected:
    virtual void Tick() override;

  private:
    builtin_interfaces::msg::Time GetTimestamp();

  private:
    ChSystem* m_system;

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr m_publisher;
};
}  // namespace ros
}  // namespace chrono

#endif