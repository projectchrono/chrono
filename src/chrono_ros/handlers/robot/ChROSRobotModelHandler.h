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
// Handler that publishes a robot model (URDF) on /robot_description for RViz.
//
// =============================================================================

#ifndef CH_ROS_ROBOT_MODEL_HANDLER_H
#define CH_ROS_ROBOT_MODEL_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChConfigROS.h"
#include "chrono_ros/ChROSHandler.h"

#ifdef CHRONO_HAS_URDF
    #include "chrono_parsers/ChParserURDF.h"
#endif

#include <memory>
#include <string>

namespace chrono {
namespace ros {

class ChROSPublisher;

/// @addtogroup ros_handlers
/// @{

/// Publishes a robot model (a URDF string) on /robot_description as
/// std_msgs/msg/String with latched (transient-local) QoS, so late-joining
/// consumers such as RViz still receive it. The model is static, so it is
/// published once and the latched QoS handles late subscribers. Schema-driven
/// port of the 9.0 handler; the string-ctor interface is unchanged. A second
/// convenience ctor takes a ChParserURDF and resolves mesh URIs (below).
class CH_ROS_API ChROSRobotModelHandler : public ChROSHandler {
  public:
    /// @param robot_model the robot description (URDF) string to publish.
    /// @param topic_name defaults to "/robot_description".
    ChROSRobotModelHandler(const std::string& robot_model, const std::string& topic_name = "/robot_description");

#ifdef CHRONO_HAS_URDF
    /// Build the robot description from a ChParserURDF: the URDF is re-emitted
    /// with every mesh/texture filename resolved to an absolute file:// URI (as
    /// RViz requires). Wrapped for Python: the ChParserURDF argument crosses from
    /// pychrono.parsers via SWIG cross-module type sharing (see demo_ROS_urdf.py).
    ChROSRobotModelHandler(chrono::parsers::ChParserURDF& parser,
                           const std::string& topic_name = "/robot_description");
#endif

    virtual bool Initialize(ChROSBridge& bridge) override;

  protected:
    virtual void Tick(double time) override;

  private:
    const std::string m_topic_name;
    std::string m_robot_model;  // non-const: the URDF-parser ctor fills it after parsing
    std::shared_ptr<ChROSPublisher> m_publisher;
    bool m_published = false;
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
