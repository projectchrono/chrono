#ifndef VIPER_HANDLER
#define VIPER_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/viper/Viper.h"

#include "rclcpp/publisher.hpp"
#include "viper_msgs/msg/viper.hpp"

namespace chrono {
namespace ros {

class ViperHandler : public ChROSHandler {
  public:
    ViperHandler(uint64_t frequency, chrono::viper::Viper& viper);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    /// NOTE: This will only update the local m_inputs variable. The driver will receive
    /// the new commands in the Tick() function.
    void Callback(const viper_msgs::msg::Viper& msg);

  private:
    viper_msgs::msg::Viper m_msg;
    chrono::viper::Viper& m_viper;

    rclcpp::Publisher<viper_msgs::msg::Viper>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
