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
// Handler responsible for publishing information about a ChBody
//
// =============================================================================

#ifndef CH_ROS_BODY_HANDLER_H
#define CH_ROS_BODY_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/handlers/ChROSHandlerUtilities.h"

#include "chrono_ros_interfaces/msg/body.hpp"

#include "chrono/physics/ChBody.h"

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// This handler is responsible for publishing state information about a ChBody
class ChROSBodyHandler : public ChROSHandler {
  public:
    /// Constructor. body_name is used as the prefix to in the ROS topic.
    ChROSBodyHandler(double update_rate, std::shared_ptr<ChBody> body, const std::string& topic_name);

    /// Initializes the handler. Creates a publisher for the body data on the topic "~/output/<body_name>/state"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

  protected:
    virtual void Tick(double time) override;

  private:
    std::shared_ptr<ChBody> m_body;  ///< The body to publish information about

    const std::string m_topic_name;          ///< The topic name to publish the body information to
    chrono_ros_interfaces::msg::Body m_msg;  ///< The message to publish
    rclcpp::Publisher<chrono_ros_interfaces::msg::Body>::SharedPtr
        m_publisher;  ///< The publisher which data is published through
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
