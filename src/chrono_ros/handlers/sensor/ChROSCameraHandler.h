// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young, Patrick Chen
// =============================================================================
//
// ROS handler for a ChCameraSensor (publishes sensor_msgs/msg/Image).
//
// =============================================================================

#ifndef CH_ROS_CAMERA_HANDLER_H
#define CH_ROS_CAMERA_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_sensor_handlers
/// @{

/// Publishes a ChCameraSensor as sensor_msgs/msg/Image (rgba8). The camera must
/// carry a ChFilterRGBA8Access filter. The pixel buffer is moved to the message as
/// one bulk blob (zero-copy into the codec); the extraction is skipped entirely
/// when no ROS subscriber is connected.
class CH_ROS_API ChROSCameraHandler : public ChROSHandler {
  public:
    /// Tick at the sensor's own update rate.
    ChROSCameraHandler(std::shared_ptr<chrono::sensor::ChCameraSensor> camera, const std::string& topic_name);
    /// Tick at an explicit rate.
    ChROSCameraHandler(double update_rate,
                       std::shared_ptr<chrono::sensor::ChCameraSensor> camera,
                       const std::string& topic_name);

    /// Creates the Image publisher and caches the image dimensions.
    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    /// Publishes the latest frame (skipped while no subscriber is connected).
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChCameraSensor> m_camera;
    const std::string m_topic_name;
    std::shared_ptr<ChROSPublisher> m_publisher;

    unsigned int m_width = 0;   ///< image width  (sensor width  / supersample factor)
    unsigned int m_height = 0;  ///< image height (sensor height / supersample factor)
    unsigned int m_step = 0;    ///< row stride in bytes (4 * width for rgba8)
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
