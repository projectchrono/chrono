#ifndef CH_ROS_VEHICLE_HANDLER
#define CH_ROS_VEHICLE_HANDLER

#include "chrono_vehicle/ChVehicle.h"

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros_interfaces/msg/ch_vehicle.hpp"

#include "rclcpp/publisher.hpp"

namespace chrono {
namespace ros {

class ChROSVehicleHandler : public ChROSHandler {
  public:
    ChROSVehicleHandler(double update_rate, chrono::vehicle::ChVehicle& vehicle);

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    virtual const std::string GetNamespace() const override { return "ChROSVehicleHandler"; }

  protected:
    virtual void Tick() override;

  private:
    chrono::vehicle::ChVehicle& m_vehicle;

    rclcpp::Publisher<chrono_ros_interfaces::msg::ChVehicle>::SharedPtr m_publisher;
};

}  // namespace ros
}  // namespace chrono

#endif
