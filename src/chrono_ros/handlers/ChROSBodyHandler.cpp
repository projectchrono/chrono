#include "chrono_ros/handlers/ChROSBodyHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

namespace chrono {
namespace ros {

ChROSBodyHandler::ChROSBodyHandler(uint64_t frequency, std::shared_ptr<ChBody> body) : ChROSHandler(frequency), m_body(body) {}

bool ChROSBodyHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", m_body->GetName(), "state");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = node->create_publisher<chrono_ros_interfaces::msg::Body>(topic_name, 1);

    m_msg.header.frame_id = m_body->GetName();

    return true;
}

void ChROSBodyHandler::Tick(double time) {
    auto pos = m_body->GetPos();
    auto rot = m_body->GetRot();
    auto lin_vel = m_body->GetPos_dt();
    auto ang_vel = m_body->GetWvel_loc();
    auto lin_acc = m_body->GetPos_dtdt();
    auto ang_acc = m_body->GetWacc_loc();

    m_msg.header.stamp = ChROSHandlerUtilities::GetROSTimestamp(time);

    m_msg.pose.position.x = pos[0];
    m_msg.pose.position.y = pos[1];
    m_msg.pose.position.z = pos[2];

    m_msg.pose.orientation.x = rot[0];
    m_msg.pose.orientation.y = rot[1];
    m_msg.pose.orientation.z = rot[2];
    m_msg.pose.orientation.w = rot[3];

    m_msg.twist.linear.x = lin_vel[0];
    m_msg.twist.linear.y = lin_vel[1];
    m_msg.twist.linear.z = lin_vel[2];

    m_msg.twist.angular.x = ang_vel[0];
    m_msg.twist.angular.y = ang_vel[1];
    m_msg.twist.angular.z = ang_vel[2];

    m_msg.accel.linear.x = lin_acc[0];
    m_msg.accel.linear.y = lin_acc[1];
    m_msg.accel.linear.z = lin_acc[2];

    m_msg.accel.angular.x = ang_acc[0];
    m_msg.accel.angular.y = ang_acc[1];
    m_msg.accel.angular.z = ang_acc[2];

    m_publisher->publish(m_msg);
}

}  // namespace ros
}  // namespace chrono
