#include "chrono_ros/handlers/ChROSDriverInputsHandler.h"

using namespace chrono::vehicle;

namespace chrono {
namespace ros {

ChROSDriverInputsHandler::ChROSDriverInputsHandler(uint64_t frequency, std::shared_ptr<ChDriver> driver)
    : ChROSHandler(frequency), m_inputs({0, 0, 0, 0}), m_driver(driver) {}

bool ChROSDriverInputsHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    m_subscription = node->create_subscription<chrono_ros_interfaces::msg::ChDriverInputs>(
        ResolveROSName("driver_inputs"), 1, std::bind(&ChROSDriverInputsHandler::Callback, this));

    return true;
}

void ChROSDriverInputsHandler::Callback(const chrono_ros_interfaces::msg::ChDriverInputs::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_inputs.steering = msg->steering;
    m_inputs.throttle = msg->throttle;
    m_inputs.braking = msg->braking;
}

void ChROSDriverInputsHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_driver.SetSteering(m_inputs.steering);
    m_driver.SetThrottle(m_inputs.throttle);
    m_driver.SetBraking(m_inputs.braking);
}

}  // namespace ros
}  // namespace chrono
