#ifndef VIPER_DC_MOTOR_CONTROL_HANDLER_H
#define VIPER_DC_MOTOR_CONTROL_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/viper/Viper.h"

#include "rclcpp/subscription.hpp"
#include "viper_msgs/msg/viper_dc_motor_control.hpp"

#include <mutex>

namespace chrono {
namespace ros {

class ChROSViperDCMotorControlHandler : public ChROSHandler {
  public:
    ChROSViperDCMotorControlHandler(uint64_t frequency, std::shared_ptr<chrono::viper::ViperDCMotorControl> driver);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const viper_msgs::msg::ViperDCMotorControl& msg);

  private:
    viper_msgs::msg::ViperDCMotorControl m_msg;
    std::shared_ptr<chrono::viper::ViperDCMotorControl> m_driver;

    rclcpp::Subscription<viper_msgs::msg::ViperDCMotorControl>::SharedPtr m_subscription;

    std::mutex m_mutex;
};

}  // namespace ros
}  // namespace chrono

#endif
