#include "chrono_ros/handlers/ChROSBodyHandler.h"

#include "chrono_ros/ChROSInterface.h"

namespace chrono {
namespace ros {

ChROSBodyHandler::ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body)
    : ChROSHandler(update_rate), m_body(body) {}

bool ChROSBodyHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    m_publisher = node->create_publisher<chrono_ros_interfaces::msg::ChBody>("/body", 1);

    return true;
}

void ChROSBodyHandler::Tick() {
    chrono_ros_interfaces::msg::ChBody msg;

    auto pos = m_body->GetPos();
    auto rot = m_body->GetRot();
    auto lin_vel = m_body->GetPos_dt();
    auto ang_vel = m_body->GetWvel_loc();
    auto lin_acc = m_body->GetPos_dtdt();
    auto ang_acc = m_body->GetWacc_loc();

    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    msg.pose.orientation.x = rot[0];
    msg.pose.orientation.y = rot[1];
    msg.pose.orientation.z = rot[2];
    msg.pose.orientation.z = rot[3];

    msg.twist.linear.x = lin_vel[0];
    msg.twist.linear.y = lin_vel[1];
    msg.twist.linear.z = lin_vel[2];

    msg.twist.angular.x = ang_vel[0];
    msg.twist.angular.y = ang_vel[1];
    msg.twist.angular.z = ang_vel[2];

    msg.accel.linear.x = lin_acc[0];
    msg.accel.linear.y = lin_acc[1];
    msg.accel.linear.z = lin_acc[2];

    msg.accel.angular.x = ang_acc[0];
    msg.accel.angular.y = ang_acc[1];
    msg.accel.angular.z = ang_acc[2];

    m_publisher->publish(msg);
}

}  // namespace ros
}  // namespace chrono