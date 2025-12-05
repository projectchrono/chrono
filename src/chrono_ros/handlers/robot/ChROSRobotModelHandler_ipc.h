#ifndef CH_ROS_ROBOT_MODEL_HANDLER_IPC_H
#define CH_ROS_ROBOT_MODEL_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct RobotModelData {
    char topic_name[128];
    uint32_t model_length;
    // The robot model string follows immediately after this struct
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
