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
// Handler that publishes transform (tf) information on /tf.
//
// =============================================================================

#ifndef CH_ROS_TF_HANDLER_H
#define CH_ROS_TF_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChBody.h"
#include "chrono/core/ChFrame.h"

#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_handlers
/// @{

/// Publishes transforms as tf2_msgs/msg/TFMessage on the "/tf" topic (the topic
/// tf2 consumers expect). Schema-driven equivalent of the Chrono 9.0 handler;
/// the call interface (constructor and the AddTransform overloads) is unchanged.
///
/// NOTE: the URDF (AddURDF) and sensor (AddSensor) convenience overloads return
/// together with the URDF and sensor demos later in Phase 5; the body/body and
/// body/frame transforms below are the dependency-free core.
class CH_ROS_API ChROSTFHandler : public ChROSHandler {
    typedef std::pair<chrono::ChFrame<>, std::string> ChFrameTransform;
    typedef std::pair<std::shared_ptr<chrono::ChBody>, std::string> ChBodyTransform;
    typedef std::variant<ChBodyTransform, ChFrameTransform> ChROSTransform;

  public:
    ChROSTFHandler(double update_rate);

    /// Creates the /tf publisher.
    virtual bool Initialize(ChROSBridge& bridge) override;

    /// Publish a transform computed each tick from two bodies (useful when the
    /// two bodies are not connected by a link). When a frame id is empty it
    /// defaults to the corresponding body's name.
    void AddTransform(std::shared_ptr<chrono::ChBody> parent,
                      const std::string& parent_frame_id,
                      std::shared_ptr<chrono::ChBody> child,
                      const std::string& child_frame_id);

    /// Publish a fixed transform between a parent body and a child frame
    /// expressed in the parent's reference frame.
    void AddTransform(std::shared_ptr<chrono::ChBody> parent,
                      const std::string& parent_frame_id,
                      chrono::ChFrame<double> child_frame,
                      const std::string& child_frame_id);

  protected:
    /// Recompute and publish all registered transforms.
    virtual void Tick(double time) override;

  private:
    std::vector<std::pair<ChROSTransform, ChROSTransform>> m_transforms;
    std::shared_ptr<ChROSPublisher> m_publisher;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
