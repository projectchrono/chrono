// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Aaron Young
// =============================================================================
//
// ROS Handler for communicating camera information
//
// =============================================================================

#ifndef CH_ROS_CAMERA_HANDLER
#define CH_ROS_CAMERA_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_sensor/sensors/ChCameraSensor.h"

#include "rclcpp/publisher.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace chrono {
namespace ros {

/// @addtogroup ros_sensor_handlers
/// @{

/// This handler is responsible for interfacing a ChCameraSensor to ROS. Will publish sensor_msgs::msg::Image.
class CH_ROS_API ChROSCameraHandler : public ChROSHandler {
  public:
    /// Constructor. The update rate is set to camera->GetUpdateRate().
    /// The update rate corresponds to the sensor's update rate.
    ChROSCameraHandler(std::shared_ptr<chrono::sensor::ChCameraSensor> camera, const std::string& topic_name);

    /// Full constructor. Takes a ChCameraSensor, update rate, and topic name.
    ChROSCameraHandler(double update_rate,
                       std::shared_ptr<chrono::sensor::ChCameraSensor> camera,
                       const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChCameraSensor> m_camera;  ///< handle to the camera sensor

    const std::string m_topic_name;                                     ///< name of the topic to publish to
    sensor_msgs::msg::Image m_image;                                    ///< the image message to publish
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;  ///< the publisher for the image message
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
