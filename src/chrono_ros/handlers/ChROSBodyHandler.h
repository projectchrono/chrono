#ifndef CH_ROS_BODY_HANDLER_H
#define CH_ROS_BODY_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChBody.h"

#include "rclcpp/publisher.hpp"
#include "chrono_ros_interfaces/msg/body.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// This handler is responsible for publishing state information about a ChBody
class ChROSBodyHandler : public ChROSHandler {
  public:
    /// Constructor. body_name is used as the prefix to in the ROS topic. The topic defaults to "~/output/<body_name>/state"
    ChROSBodyHandler(uint64_t frequency, std::shared_ptr<ChBody> body);

    /// Initializes the handler. Creates a publisher for the body data on the topic "~/output/<body_name>/state"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<ChBody> m_body;

    chrono_ros_interfaces::msg::Body m_msg;

    rclcpp::Publisher<viper_msgs::msg::Body>::SharedPtr m_publisher;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
