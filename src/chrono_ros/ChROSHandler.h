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
// Base class for all ros handlers
//
// =============================================================================

#ifndef CH_ROS_HANDLER_H
#define CH_ROS_HANDLER_H

#include "chrono_ros/ChApiROS.h"
#include "chrono_ros/ChROSInterface.h"
#include "chrono_ros/ipc/ChROSIPCMessage.h"

#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <cstdint>
#include <cstring>

// Forward declare IPC message type to avoid circular dependency
namespace chrono {
namespace ros {
namespace ipc {
    struct Message;
}
}
}

namespace chrono {
namespace ros {

/// @addtogroup ros_handlers
/// @{

/// Base class for a ROS handler. A specific handler should inherit from here.
/// A handler is essentially a wrapper around a ROS subscriber/publisher/service/action.
///
/// IMPORTANT: To support IPC communication while avoiding VSG symbol collisions, handlers
/// should be split into two logical parts:
/// 1. Data extraction from Chrono (runs in main process, NO ROS symbols)
/// 2. ROS publishing/subscribing (runs in subprocess or direct mode, full ROS API)
///
/// This split is transparent to the handler implementation - the framework automatically
/// handles serialization and IPC when needed. Handlers define a data struct and implement
/// methods to extract/apply data and publish/subscribe.
class CH_ROS_API ChROSHandler {
  public:
    /// Destructor for the ChROSHandler
    virtual ~ChROSHandler() = default;

    /// Initializes the handler. Must be implemented by derived classes. This is called after rclcpp::init().
    /// In IPC mode, this is only called in the subprocess. In direct mode, called in main process.
    /// Here the underlying ROS objects (e.g. publisher, subscription) should be created.
    /// @param interface The interface to the ROS node
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) = 0;

    /// Get the period which this handler operates at
    const double GetUpdateRate() const { return m_update_rate; }

    /// Get the number of times Tick() has been called
    const uint64_t GetTickCount() const { return m_tick_count; }
    
    /// Check if this handler is a publisher (data flows from Chrono to ROS)
    /// Default is true. Override to false for subscribers.
    virtual bool IsPublisher() const { return true; }
    
    /// Get the message type of this handler.
    /// Must be implemented by derived classes to identify the IPC message type.
    /// @return The IPC message type for this handler
    typedef chrono::ros::ipc::MessageType MessageType;
    virtual chrono::ros::ipc::MessageType GetMessageType() const { return static_cast<chrono::ros::ipc::MessageType>(0); }

    /// Get serialized data from this handler for IPC transmission.
    /// Framework calls this in main process for publishers.
    /// Handler should extract Chrono data and return as byte vector.
    /// Default implementation calls Tick() for backward compatibility.
    /// @param time Current simulation time
    /// @return Serialized data as byte vector (empty if not supported)
    virtual std::vector<uint8_t> GetSerializedData(double time) {
        return std::vector<uint8_t>();  // Empty by default
    }
    
    /// Publish data to ROS from serialized bytes.
    /// Framework calls this in subprocess (or direct mode) for publishers.
    /// Handler should deserialize and publish to ROS topics.
    /// Default implementation does nothing for backward compatibility.
    /// @param data Serialized data bytes
    /// @param interface ROS interface to use for publishing
    virtual void PublishFromSerialized(const std::vector<uint8_t>& data, 
                                       std::shared_ptr<ChROSInterface> interface) {
        // Default: no-op for backward compatibility
    }
    
    /// Apply data to Chrono from serialized bytes.
    /// Framework calls this in main process for subscribers.
    /// Handler should deserialize and apply to Chrono objects.
    /// Default implementation does nothing for backward compatibility.
    /// @param data Serialized data bytes
    virtual void ApplyFromSerialized(const std::vector<uint8_t>& data) {
        // Default: no-op for backward compatibility
    }
    
    /// Setup subscriber in subprocess to send data back to main process.
    /// Framework calls this in subprocess for subscribers.
    /// Handler should create ROS subscription and call callback with serialized data.
    /// Default implementation does nothing for backward compatibility.
    /// @param interface ROS interface to use for subscribing
    /// @param callback Function to call with serialized data when ROS message arrives
    virtual void SubscribeAndForward(std::shared_ptr<ChROSInterface> interface,
                                     std::function<void(const std::vector<uint8_t>&)> callback) {
        // Default: no-op for backward compatibility
    }
    
    /// Handle incoming IPC message from ROS subscriber (bidirectional communication).
    /// Override this in handlers that receive data from ROS via IPC.
    /// Base implementation does nothing - only bidirectional subscribers override this.
    /// @param msg The incoming IPC message to process
    virtual void HandleIncomingMessage(const ipc::Message& msg) {
        // Default: do nothing - only bidirectional subscribers override this
    }
    
    /// Returns true if this handler processes incoming IPC messages.
    /// Override to return true in bidirectional subscriber handlers.
    virtual bool SupportsIncomingMessages() const { return false; }

  protected:
    /// Constructor for the ChROSHandler
    /// @param update_rate Update rate with which the handler should tick relative to the simulation clock. NOTE: A
    ///   update_rate of 0 indicates tick should be called on each update of the simulation.
    explicit ChROSHandler(double update_rate);

    /// Increment the tick count
    void IncrementTickCount() { m_tick_count++; }

  private:
    const double m_update_rate;  ///< Update rate of the handler
    uint64_t m_tick_count;  ///< Number of times Tick() has been called
    double m_time_elapsed_since_last_tick;  ///< Time elapsed since last tick
};

/// @} ros_handlers

}  // namespace ros
}  // namespace chrono

#endif