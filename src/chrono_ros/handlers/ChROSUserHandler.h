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
// Template base class for user-defined handlers to simplify IPC serialization
//
// =============================================================================

#ifndef CH_ROS_USER_HANDLER_H
#define CH_ROS_USER_HANDLER_H

#include "chrono_ros/ChROSHandler.h"
#include <cstring>
#include <vector>

namespace chrono {
namespace ros {

/// Template base class for user-defined handlers that simplifies IPC serialization.
/// Users define a POD (Plain Old Data) struct for their data, and this class handles
/// the serialization/deserialization automatically.
///
/// @tparam DataStruct A POD struct containing the data to be transmitted
template <typename DataStruct>
class ChROSUserHandler : public ChROSHandler {
  public:
    /// Constructor
    /// @param update_rate Update rate in Hz
    ChROSUserHandler(double update_rate) : ChROSHandler(update_rate) {}
    
    virtual ~ChROSUserHandler() = default;

    /// Main Process: Implement this to fill the data struct from Chrono simulation.
    /// @param data The struct to fill
    /// @param time Current simulation time
    /// @return true if data should be sent, false otherwise
    virtual bool FillData(DataStruct& data, double time) = 0;

    /// Subprocess: Implement this to publish the data to ROS.
    /// @param data The deserialized struct received from main process
    /// @param interface The ROS interface
    virtual void PublishData(const DataStruct& data, std::shared_ptr<ChROSInterface> interface) = 0;

    // -------------------------------------------------------------------------
    // Implementation of ChROSHandler IPC methods
    // -------------------------------------------------------------------------

    virtual std::vector<uint8_t> GetSerializedData(double time) override {
        DataStruct data;
        // Initialize with zeros to avoid sending uninitialized memory
        std::memset(&data, 0, sizeof(DataStruct));
        
        if (!FillData(data, time)) {
            return std::vector<uint8_t>();
        }
        
        std::vector<uint8_t> buffer(sizeof(DataStruct));
        std::memcpy(buffer.data(), &data, sizeof(DataStruct));
        return buffer;
    }

    virtual void PublishFromSerialized(const std::vector<uint8_t>& serialized_data, 
                                     std::shared_ptr<ChROSInterface> interface) override {
        if (serialized_data.size() != sizeof(DataStruct)) {
            // Size mismatch - ignore
            return;
        }
        
        DataStruct data;
        std::memcpy(&data, serialized_data.data(), sizeof(DataStruct));
        PublishData(data, interface);
    }
    
    // Note: Derived classes must still implement:
    // - Initialize()
    // - GetMessageType()
    // - Tick() (usually empty for IPC handlers)
};

}  // namespace ros
}  // namespace chrono

#endif
