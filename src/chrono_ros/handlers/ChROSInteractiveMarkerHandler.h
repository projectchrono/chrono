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
// Handler responsible for running the InterativeMarkerServer
//
// =============================================================================

#ifndef CH_ROS_INTERACTIVE_MARKER_HANDLER_H
#define CH_ROS_INTERACTIVE_MARKER_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChLinkMotor.h"

#include "interactive_markers/interactive_marker_server.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

#ifdef CHRONO_PARSERS_URDF
    #include "chrono_parsers/ChParserURDF.h"
#endif

#include <vector>
#include <utility>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

enum Axis { X, Y, Z };

struct MarkerLink {
    Axis axis;
    double setpoint;
    std::shared_ptr<chrono::ChLinkMotor> motor;

    visualization_msgs::msg::InteractiveMarker marker;
    std::function<void(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr)> callback;
};

/// This handler is responsible for publishing state information about a ChBody
class ChROSInteractiveMarkerHandler : public ChROSHandler {
  public:
    /// Constructor.
    ChROSInteractiveMarkerHandler(double update_rate);

    /// Initializes the handler. Creates a publisher for the body data on the topic "~/output/<body_name>/state"
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    void AddMotor(std::shared_ptr<chrono::ChLinkMotor> motor, Axis axis, const std::string& frame_id);

#ifdef CHRONO_PARSERS_URDF
    void AddURDF(chrono::parsers::ChParserURDF& parser);
#endif

  protected:
    virtual void Tick(double time) override;

  private:
    void ProcessFeedback(visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr feedback);

  private:
    std::unordered_map<std::string, MarkerLink> m_marker_links;
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> m_marker_server;  ///< The marker server
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
