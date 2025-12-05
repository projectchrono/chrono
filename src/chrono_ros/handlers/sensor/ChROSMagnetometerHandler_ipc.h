#ifndef CH_ROS_MAGNETOMETER_HANDLER_IPC_H
#define CH_ROS_MAGNETOMETER_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct MagnetometerData {
    char topic_name[128];
    char frame_id[128];
    double magnetic_field[3];
    double magnetic_field_covariance[9];
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
