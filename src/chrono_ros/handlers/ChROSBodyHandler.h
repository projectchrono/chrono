#ifndef CH_ROS_BODY_HANDLER
#define CH_ROS_BODY_HANDLER

#include "chrono/physics/ChBody.h"

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros_interfaces/msg/ch_body.hpp"

#include "rclcpp/publisher.hpp"

namespace chrono {
namespace ros {
class ChROSBodyHandler : public ChROSHandler {
  public:
    ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual const std::string GetNamespace() const override { return "ChROSBodyHandler"; }

  protected:
    virtual void Tick() override;

  private:
    std::shared_ptr<ChBody> m_body;

    rclcpp::Publisher<chrono_ros_interfaces::msg::ChBody>::SharedPtr m_publisher;
};
}  // namespace ros
}  // namespace chrono

#endif
