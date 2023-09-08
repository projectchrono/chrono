#include "chrono_ros/handlers/ChROSCommandHandler.h"

#include "chrono/core/ChLog.h"

#include <functional>

using namespace std::placeholders;

namespace chrono {
namespace ros {

ChROSCommandHandler::ChROSCommandHandler(uint64_t frequency)
    : ChROSHandler(frequency),
      m_start({false, false}),
      m_started({false, false}),
      m_stop({false, false}),
      m_reset({false, false}) {}

bool ChROSCommandHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    m_start_service = node->create_service<chrono_ros_interfaces::srv::ChStartSimulation>(
        "~/command/start", std::bind(&ChROSCommandHandler::StartCallback, this, _1, _2));
    m_stop_service = node->create_service<chrono_ros_interfaces::srv::ChStopSimulation>(
        "~/command/stop", std::bind(&ChROSCommandHandler::StopCallback, this, _1, _2));
    m_reset_service = node->create_service<chrono_ros_interfaces::srv::ChResetSimulation>(
        "~/command/reset", std::bind(&ChROSCommandHandler::ResetCallback, this, _1, _2));

    return true;
}

void ChROSCommandHandler::Tick(double time) {
    std::unique_lock<std::mutex> lock(m_mutex);

    m_start.public_status = m_start.private_status;
    m_stop.public_status = m_stop.private_status;
    m_reset.public_status = m_reset.private_status;

    m_started.private_status = m_started.public_status;
}

void ChROSCommandHandler::StartCallback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response) {
    GetLog() << "Got ChStartSimulation command.\n";

    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_started.private_status) {
        lock.unlock();
        GetLog() << "Simulation already started. Ignoring...\n";
        response->success = false;
        return;
    }

    m_start.private_status = true;
    response->success = true;
}

void ChROSCommandHandler::StopCallback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChStopSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChStopSimulation::Response> response) {
    GetLog() << "Got ChStopSimulation command.\n";

    std::unique_lock<std::mutex> lock(m_mutex);
    if (!m_started.private_status) {
        lock.unlock();
        GetLog() << "Simulation can't be stopped because it was never started. Ignoring...\n";
        response->success = false;
        return;
    }

    m_stop.private_status = true;
    response->success = true;
}

void ChROSCommandHandler::ResetCallback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChResetSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChResetSimulation::Response> response) {
    GetLog() << "Got ChResetSimulation command.\n";
}

bool ChROSCommandHandler::ShouldStart() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_start.public_status;
}

bool ChROSCommandHandler::ShouldStop() {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_stop.public_status;
}

void ChROSCommandHandler::SetStarted(bool started) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_started.public_status = started;
}

}  // namespace ros
}  // namespace chrono