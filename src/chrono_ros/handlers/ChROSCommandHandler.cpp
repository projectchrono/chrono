#include "chrono_ros/handlers/ChROSCommandHandler.h"

#include <placeholders>

using namespace std::placeholders;

namespace chrono {
namespace ros {

ChROSCommandHandler::ChROSCommandHandler()
    : ChROSHandler(), m_start(false), m_started(false), m_stop(false), m_reset(false), {}

bool ChROSCommandHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    m_start_service = node->create_service<chrono_ros_interfaces::srv::ChStartSimulation>(
        ResolveROSName("start"), std::bind(&ChROSCommandHandler::StartCallback, this, _1, _2));
    m_stop_service = node->create_service<chrono_ros_interfaces::srv::ChStopSimulation>(
        ResolveROSName("stop"), std::bind(&ChROSCommandHandler::StopCallback, this, _1, _2));
    m_reset_service = node->create_service<chrono_ros_interfaces::srv::ChResetSimulation>(
        ResolveROSName("reset"), std::bind(&ChROSCommandHandler::ResetCallback, this, _1, _2));

    return true;
}

void ChROSCommandHandler::Tick(double time) {}

void ChROSCommandHandler::StartCallback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response) {
    GetLog() << "Got ChStartSimulation command.\n";

    std::unique_lock<std::mutex> lock(m_mutex);
    if (m_started) {
        lock.unlock();
        GetLog() << "Simulation already started. Ignoring...\n";
        response->success = false;
        return;
    }

    m_start = true;
    response->success = true;
}

void ChROSCommandHandler::StopCallback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response) {
    GetLog() << "Got ChStopSimulation command.\n";

    std::unique_lock<std::mutex> lock(m_mutex);
    if (!m_started) {
        lock.unlock();
        GetLog() << "Simulation can't be stopped because it was never started. Ignoring...\n";
        response->success = false;
        return;
    }

    m_stop = true;
    response->success = true;
}

void ChROSCommandHandler::ResetCallback(
    const std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Request> request,
    std::shared_ptr<chrono_ros_interfaces::srv::ChStartSimulation::Response> response) {
    GetLog() << "Got ChResetSimulation command.\n";
}

}  // namespace ros
}  // namespace chrono