#ifndef CH_ROS_ACCELEROMETER_HANDLER_IPC_H
#define CH_ROS_ACCELEROMETER_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct AccelerometerData {
    char topic_name[128];
    char frame_id[128];
    double linear_acceleration[3];
    double linear_acceleration_covariance[9];
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
