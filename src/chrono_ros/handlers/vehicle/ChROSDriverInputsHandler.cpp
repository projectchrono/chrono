#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

using std::placeholders::_1;

using namespace chrono::vehicle;

namespace chrono {
namespace ros {

ChROSDriverInputsHandler::ChROSDriverInputsHandler(uint64_t frequency, std::shared_ptr<ChDriver> driver)
    : ChROSHandler(frequency), m_inputs({0, 0, 0, 0}), m_driver(driver) {}

bool ChROSDriverInputsHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("input", "driver_inputs");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_subscription = node->create_subscription<chrono_ros_interfaces::msg::ChDriverInputs>(
        topic_name, 1, std::bind(&ChROSDriverInputsHandler::Callback, this, _1));

    return true;
}

void ChROSDriverInputsHandler::Callback(const chrono_ros_interfaces::msg::ChDriverInputs& msg) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_inputs.m_steering = msg.steering;
    m_inputs.m_throttle = msg.throttle;
    m_inputs.m_braking = msg.braking;
}

void ChROSDriverInputsHandler::Tick(double time) {
    std::lock_guard<std::mutex> lock(m_mutex);

    m_driver->SetSteering(m_inputs.m_steering);
    m_driver->SetThrottle(m_inputs.m_throttle);
    m_driver->SetBraking(m_inputs.m_braking);
}

}  // namespace ros
}  // namespace chrono
