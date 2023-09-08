#ifndef CH_ROS_COMMAND_HANDLER
#define CH_ROS_COMMAND_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_ros_interfaces/srv/ch_start_simulation.hpp"
#include "chrono_ros_interfaces/srv/ch_stop_simulation.hpp"
#include "chrono_ros_interfaces/srv/ch_reset_simulation.hpp"

#include "rclcpp/service.hpp"

#include <mutex>

namespace chrono {
namespace ros {

struct PublicPrivateStatus {
    bool public_status, private_status;
};

class ChROSCommandHandler : public ChROSHandler {
  public:
    ChROSCommandHandler(uint64_t frequency = 0);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    bool ShouldStart();
    bool ShouldStop();

    void SetStarted(bool started);

  protected:
    virtual void Tick(double time) override;

  private:
    void StartCallback(const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
                       std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response);

    void StopCallback(const std::shared_ptr<chrono_ros_interfaces::srv::ChStopSimulation::Request> request,
                      std::shared_ptr<chrono_ros_interfaces::srv::ChStopSimulation::Response> response);
    void ResetCallback(const std::shared_ptr<chrono_ros_interfaces::srv::ChResetSimulation::Request> request,
                       std::shared_ptr<chrono_ros_interfaces::srv::ChResetSimulation::Response> response);

    rclcpp::Service<chrono_ros_interfaces::srv::ChStartSimulation>::SharedPtr m_start_service;
    rclcpp::Service<chrono_ros_interfaces::srv::ChStopSimulation>::SharedPtr m_stop_service;
    rclcpp::Service<chrono_ros_interfaces::srv::ChResetSimulation>::SharedPtr m_reset_service;

    std::mutex m_mutex;

    PublicPrivateStatus m_start, m_started, m_stop, m_reset;
};

}  // namespace ros
}  // namespace chrono

#endif
