// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Handler responsible for publishing transform (tf) information
//
// =============================================================================

#ifndef CH_ROS_TF_HANDLER_H
#define CH_ROS_TF_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono/physics/ChBody.h"

#include "tf2_ros/transform_broadcaster.h"

#ifdef CHRONO_PARSERS_URDF
    #include "chrono_parsers/ChParserURDF.h"
#endif
#ifdef CHRONO_SENSOR
    #include "chrono_sensor/sensors/ChSensor.h"
#endif

#include <vector>
#include <utility>
#include <variant>

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// @brief Handler responsible for publishing transform (tf) information via IPC.
///
/// PUBLISHER PATTERN (variable-size data):
/// - Main process: Calls GetSerializedData() each tick → packs multiple transforms → sends IPC
/// - Subprocess: Receives IPC → unpacks transform array → publishes to /tf
///
/// This handler supports dynamic transform lists (body-to-body) and static transforms (body-to-frame).
/// All transforms are computed in main process and sent as serialized data to subprocess.
///
/// IPC data structures defined in ChROSIPCMessage.h:
/// - ipc::TFData: Header with transform count
/// - ipc::TFTransform: Individual transform data (parent/child frames, position, rotation)
///
/// Implementation files:
/// - ChROSTFHandler.cpp: Main process logic (compute transforms, serialize)
/// - ChROSTFHandler_ros.cpp: Subprocess ROS publishing (deserialize, publish tf)
class CH_ROS_API ChROSTFHandler : public ChROSHandler {
    typedef std::pair<chrono::ChFrame<>, std::string> ChFrameTransform;
    typedef std::pair<std::shared_ptr<chrono::ChBody>, std::string> ChBodyTransform;
    typedef std::variant<ChBodyTransform, ChFrameTransform> ChROSTransform;

  public:
    /// Constructor.
    ChROSTFHandler(double update_rate);

    /// @brief Initializes the handler (no-op in IPC mode, broadcaster in subprocess)
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;

    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::TF_DATA; }

    /// Extract transform data for IPC transmission to subprocess
    /// Computes all transforms, serializes into variable-size byte array
    /// Format: ipc::TFData header + array of ipc::TFTransform structs (see ChROSIPCMessage.h)
    /// @param time Current simulation time in seconds
    /// @return Serialized transform data
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

    /// @brief Add a transform to be published. This version of the AddTransform function will use two bodies directly
    /// and calculate the transform at each tick. This is useful for two bodies that are not connected by a link.
    /// For each iteration of this method, when the frame id for the parent and/or child is not passed, it defaults
    /// to the name of the body.
    void AddTransform(std::shared_ptr<chrono::ChBody> parent,
                      const std::string& parent_frame_id,
                      std::shared_ptr<chrono::ChBody> child,
                      const std::string& child_frame_id);

    /// @brief Add a transform to be published. This version of the AddTransform function will publish a static
    /// transform between the parent and child frame. This is useful for two bodies that are connected by a link.
    /// @param parent The parent body
    /// @param parent_frame The parent frame id.
    /// @param child_frame The child frame
    /// @param child_frame_id The child frame id
    void AddTransform(std::shared_ptr<chrono::ChBody> parent,
                      const std::string& parent_frame_id,
                      chrono::ChFrame<double> child_frame,
                      const std::string& child_frame_id);

#ifdef CHRONO_PARSERS_URDF
    /// @brief Add a transform to be published from a URDF file. This method will step through the kinematic tree of the
    /// passed URDF parser and add transforms for each link.
    /// @param parser The URDF parser
    void AddURDF(chrono::parsers::ChParserURDF& parser);
#endif

#ifdef CHRONO_SENSOR
    /// @brief Add a transform to be published from a sensor. This is simply an alias to
    /// AddTransform(sensor->GetParent(), sensor->GetOffsetPose(), frame_id).
    /// @param sensor The sensor
    /// @param parent_frame_id The parent frame id
    /// @param child_frame_id The child frame id
    void AddSensor(std::shared_ptr<chrono::sensor::ChSensor> sensor,
                   const std::string& parent_frame_id,
                   const std::string& child_frame_id);
#endif

  private:
    std::vector<std::pair<ChROSTransform, ChROSTransform>> m_transforms;  ///< The transforms to publish

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;  ///< The tf broadcaster
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif
