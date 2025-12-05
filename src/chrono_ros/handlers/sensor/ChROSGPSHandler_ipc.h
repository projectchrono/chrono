#ifndef CH_ROS_GPS_HANDLER_IPC_H
#define CH_ROS_GPS_HANDLER_IPC_H

#include "chrono_ros/ipc/ChROSIPCMessage.h"

namespace chrono {
namespace ros {
namespace ipc {

struct GPSData {
    char topic_name[128];
    char frame_id[128];
    double time;
    double latitude;
    double longitude;
    double altitude;
    double position_covariance[9];
};

}  // namespace ipc
}  // namespace ros
}  // namespace chrono

#endif
