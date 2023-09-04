#ifndef CH_ROS_COMMAND_HANDLER
#define CH_ROS_COMMAND_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_ros_interfaces/srv/ch_start_simulation.h"
#include "chrono_ros_interfaces/srv/ch_stop_simulation.h"

#include "rclcpp/service.hpp"

#include <mutex>

namespace chrono {
namespace ros {

class ChROSCommandHandler : public ChROSHandler {
  public:
    ChROSCommandHandler();

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    void StartCallback(const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
                       std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response);

    void StopCallback(const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
                      std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response);
    void ResetCallback(const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
                       std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response);

    rclcpp::Service<chrono_ros_interfaces::srv::ChStartSimulation>::SharedPtr m_start_service;
    rclcpp::Service<chrono_ros_interfaces::srv::ChStopSimulation>::SharedPtr m_stop_service;
    rclcpp::Service<chrono_ros_interfaces::srv::ChResetSimulation>::SharedPtr m_reset_service;

    std::mutex m_mutex;

    bool m_start, m_started;
    bool m_stop;
    bool m_reset;
};

}  // namespace ros
}  // namespace chrono

#endif
