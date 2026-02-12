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
// Handler responsible for receiving and updating the DC motor control commands
// for the Viper rover
//
// =============================================================================

#ifndef CH_VIPER_DC_MOTOR_CONTROL_HANDLER_H
#define CH_VIPER_DC_MOTOR_CONTROL_HANDLER_H

#include "chrono_ros/ChROSHandler.h"

#include "chrono_models/robot/viper/Viper.h"

// IPC data structure (separate header - safe for subprocess to include)
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler_ipc.h"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_robot_handlers
/// @{

/// This handler is responsible for interfacing a ViperDCMotorControl driver to ROS. Will instantiate a subscriber to
/// chrono_ros_interfaces::msg::ViperDCMotorControl.
class CH_ROS_API ChROSViperDCMotorControlHandler : public ChROSHandler {
  public:
    /// Constructor. Takes a ViperDCMotorControl driver
    ChROSViperDCMotorControlHandler(double update_rate,
                                    std::shared_ptr<chrono::viper::ViperDCMotorControl> driver,
                                    const std::string& topic_name);

    /// Initializes the handler.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    
    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::VIPER_DC_MOTOR_CONTROL; }

    /// Apply motor control inputs received from ROS (used in IPC mode)
    void ApplyInputs(const ipc::ViperDCMotorControlData& data);
    
    /// Handle incoming IPC message from ROS subscriber (bidirectional)
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;
    
    /// This handler receives incoming messages
    virtual bool SupportsIncomingMessages() const override { return true; }

  protected:
    /// For IPC mode: sends topic name to subprocess once to create subscriber
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    std::shared_ptr<chrono::viper::ViperDCMotorControl> m_driver;  ///< handle to the driver

    const std::string m_topic_name;                         ///< name of the topic to publish to
    ipc::ViperDCMotorControlData m_inputs;                  ///< stored inputs
    bool m_subscriber_setup_sent;  ///< tracks if setup message was sent to subprocess

    std::shared_ptr<void> m_subscription; /// < Type erased pointer to the subscription

    std::mutex m_mutex;
};

/// @} ros_robot_handlers

}  // namespace ros
}  // namespace chrono

#endif
