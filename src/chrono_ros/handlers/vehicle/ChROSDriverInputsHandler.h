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
// Author: Aaron Young, Patrick Chen
// =============================================================================
//
// Handler for interfacing a ChDriver to ROS
//
// =============================================================================

#ifndef CH_ROS_DRIVER_INPUTS_HANDLER
#define CH_ROS_DRIVER_INPUTS_HANDLER

#include "chrono_ros/ChROSHandler.h"

#include "chrono_vehicle/ChDriver.h"

// IPC data structure (separate header - safe for subprocess to include)
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler_ipc.h"

#include <mutex>

namespace chrono {
namespace ros {

/// @addtogroup ros_vehicle_handlers
/// @{

// =============================================================================
// HANDLER CLASS (main process)
// =============================================================================

/// Handler for interfacing a ChDriver to ROS via bidirectional IPC communication.
///
/// BIDIRECTIONAL SUBSCRIBER PATTERN:
/// - Main process: Calls GetSerializedData() once to send topic name → subprocess creates subscriber
/// - Subprocess: ROS message arrives → packs IPC data → sends back to main process
/// - Main process: Receives IPC → calls HandleIncomingMessage() → applies data to Chrono object
///
/// Data flow:
/// External ROS → Subprocess subscriber → IPC channel → Main process → ChDriver
///
/// Implementation files:
/// - ChROSDriverInputsHandler.cpp: Main process logic (ApplyInputs, HandleIncomingMessage)
/// - ChROSDriverInputsHandler_ros.cpp: Subprocess ROS subscriber (compiled into chrono_ros_node)
///
/// To implement a similar bidirectional subscriber:
/// 1. Define IPC struct in handler header under namespace ipc (plain C++ types)
/// 2. Override SupportsIncomingMessages() to return true
/// 3. Override HandleIncomingMessage() to extract IPC data and apply to Chrono object
/// 4. Implement GetSerializedData() to send topic name once (empty afterwards)
/// 5. Create YourHandler_ros.cpp with subscriber callback that sends IPC back
/// 6. Register with CHRONO_ROS_REGISTER_HANDLER(YOUR_MESSAGE_TYPE, YourSetupFunction)
/// 7. Add YOUR_MESSAGE_TYPE to MessageType enum in ChROSIPCMessage.h
/// 8. Add handler recognition to ChROSManager::GetHandlerMessageType()
class CH_ROS_API ChROSDriverInputsHandler : public ChROSHandler {
  public:
    /// Constructor with default update rate
    /// @param driver Chrono vehicle driver to update with ROS commands
    /// @param topic_name ROS topic to subscribe to for driver inputs
    ChROSDriverInputsHandler(std::shared_ptr<chrono::vehicle::ChDriver> driver, const std::string& topic_name);

    /// Constructor with custom update rate
    /// @param update_rate Rate at which to apply received inputs to driver (Hz)
    /// @param driver Chrono vehicle driver to update with ROS commands  
    /// @param topic_name ROS topic to subscribe to for driver inputs
    ChROSDriverInputsHandler(double update_rate,
                             std::shared_ptr<chrono::vehicle::ChDriver> driver,
                             const std::string& topic_name);

    /// Initialize handler (called once at startup in main process)
    /// In IPC mode, this does nothing. Subprocess will create the actual ROS subscriber.
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    
    /// Get the message type of this handler
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::DRIVER_INPUTS; }

    /// Apply driver inputs received from subprocess via IPC
    /// This method is called internally by HandleIncomingMessage()
    /// @param steering Steering value from ROS message
    /// @param throttle Throttle value from ROS message  
    /// @param braking Braking value from ROS message
    void ApplyInputs(double steering, double throttle, double braking);
    
    /// Handle incoming IPC message from subprocess ROS subscriber
    /// Called by ChROSManager when IPC message of type DRIVER_INPUTS arrives.
    /// Extracts DriverInputsData from message payload and applies to driver.
    /// @param msg IPC message containing DriverInputsData payload
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;
    
    /// Indicates this handler receives messages from subprocess
    /// @return true (this is a bidirectional subscriber)
    virtual bool SupportsIncomingMessages() const override { return true; }

  protected:
    /// Send topic name to subprocess once to trigger subscriber creation
    /// First call: Returns topic name as bytes for subprocess setup
    /// Subsequent calls: Returns empty vector (no data to publish)
    /// @param time Current simulation time (unused for subscribers)
    /// @return Topic name bytes on first call, empty afterwards
    virtual std::vector<uint8_t> GetSerializedData(double time) override;

  private:
    std::shared_ptr<chrono::vehicle::ChDriver> m_driver;  ///< the driver to update

    const std::string m_topic_name;          ///< name of the topic to publish to
    chrono::vehicle::DriverInputs m_inputs;  ///< stores the most recent inputs
    chrono::vehicle::DriverInputs m_applied_inputs;  ///< last inputs applied to driver
    bool m_subscriber_setup_sent;  ///< tracks if setup message was sent to subprocess

    std::mutex m_mutex;  ///< used to control access to m_inputs
};

}  // namespace ros
}  // namespace chrono

#endif
