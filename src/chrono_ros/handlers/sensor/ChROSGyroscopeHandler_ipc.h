#ifndef CH_ROS_GYROSCOPE_HANDLER_IPC_H
#define CH_ROS_GYROSCOPE_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct GyroscopeData {
    char topic_name[128];
    char frame_id[128];
    double angular_velocity[3];
    double angular_velocity_covariance[9];
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
