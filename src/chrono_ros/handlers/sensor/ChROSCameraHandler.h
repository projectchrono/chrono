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
class ChROSCameraHandler : public ChROSHandler {
  public:
    /// Constructor
    ChROSCameraHandler(std::shared_ptr<chrono::sensor::ChCameraSensor> camera);

    /// Initializes the handler. Creates a publisher for the image on the topic "~/output/camera/camera.GetName()/image"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<chrono::sensor::ChCameraSensor> m_camera;

    sensor_msgs::msg::Image m_image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_publisher;
};

/// @} ros_sensor_handlers

}  // namespace ros
}  // namespace chrono

#endif
