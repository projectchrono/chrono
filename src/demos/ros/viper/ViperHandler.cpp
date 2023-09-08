#include "ViperHandler.h"

#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "viper_msgs/msg/viper_wheel_id.hpp"

using std::placeholders::_1;

using namespace chrono::ros;
using namespace chrono::viper;

namespace chrono {
namespace ros {

ViperHandler::ViperHandler(uint64_t frequency, Viper& viper) : ChROSHandler(frequency), m_viper(viper) {}

bool ViperHandler::Initialize(std::shared_ptr<ChROSInterface> interface) {
    auto node = interface->GetNode();

    auto topic_name = ChROSHandlerUtilities::BuildRelativeTopicName("output", "simulation", "viper", "state");

    if (!ChROSHandlerUtilities::CheckROSTopicName(interface, topic_name)) {
        return false;
    }

    m_publisher = node->create_publisher<viper_msgs::msg::Viper>(topic_name, 1);

    return true;
}

void ViperHandler::Tick(double time) {
    viper_msgs::msg::Viper msg;

    auto body = m_viper.GetChassis()->GetBody();

    auto pos = body->GetPos();
    auto rot = body->GetRot();
    auto lin_vel = body->GetPos_dt();
    auto ang_vel = body->GetWvel_loc();
    auto lin_acc = body->GetPos_dtdt();
    auto ang_acc = body->GetWacc_loc();

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
