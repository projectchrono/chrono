#ifndef CH_ROS_DRIVER_INPUTS_HANDLER
#define CH_ROS_DRIVER_INPUTS_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_vehicle/ChDriver.h"

#include "rclcpp/subscriber.hpp"
#include "chrono_ros_interfaces/msg/ch_driver_inputs.hpp"

#include <mutex>

namespace chrono {
namespace ros {

class ChROSDriverInputsHandler : public ChROSHandler {
  public:
    ChROSDriverInputsHandler(std::shared_ptr<chrono::vehicle::ChDriver> driver);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const chrono_ros_interfaces::msg::ChDriverInputs::SharedPtr msg);

  private:
    chrono::vehicle::DriverInputs m_inputs;
    std::shared_ptr<chrono::vehicle::ChDriver> m_driver;

    rclcpp::Subscriber<chrono_ros_interfaces::msg::ChDriverInputs>::SharedPtr m_subscriber;

    std::mutex m_mutex;
};

}  // namespace ros
}  // namespace chrono

#endif
