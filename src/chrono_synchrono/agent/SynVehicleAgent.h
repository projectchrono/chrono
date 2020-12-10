// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
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
// Base class with methods that are common to both Wheeled and Tracked
// Syn_VehicleAgents, in particular the fact that they use a ChDriver to decide
// how to navigate and that they know about terrain
//
// =============================================================================

#ifndef SYN_VEHICLE_AGENT_H
#define SYN_VEHICLE_AGENT_H

#include "chrono_synchrono/SynApi.h"
#include "chrono_synchrono/agent/SynAgent.h"
#include "chrono_synchrono/utils/SynUtilsJSON.h"
#include "chrono_synchrono/vehicle/SynVehicle.h"
#include "chrono_synchrono/terrain/SynTerrain.h"
#include "chrono_synchrono/brain/SynVehicleBrain.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_agent
/// @{

/// Base class for wheeled and tracked SynVehicleAgents. Both know about ChTerrains, ChDrivers and ChVehicleBrains
class SYN_API SynVehicleAgent : public SynAgent {
  public:
    /// Construct a vehicle agent with the specified rank and system
    /// Underlying agent is set to a vehicle type automatically

    ///@brief Construct a vehicle agent with the specified rank and system
    /// Underlying agent is set to a vehicle type automatically
    ///
    ///@param rank the rank of this agent
    ///@param system an optional argument of a ChSystem to build zombies and get the Time
    SynVehicleAgent(unsigned int rank, ChSystem* system = 0);

    ///@brief Destructor
    virtual ~SynVehicleAgent() {}

    ///@brief Synchronize the underlying vehicle
    ///
    ///@param time the time to synchronize to
    ///@param driver_inputs the driver inputs (i.e. throttle, braking, steering)
    virtual void Synchronize(double time, vehicle::ChDriver::Inputs driver_inputs) = 0;

    ///@brief Initialize the zombie representation of the underlying vehicle.
    /// Simply calls the same method on the underlying vehicle.
    ///
    ///@param system the system used to add bodies to for consistent visualization
    virtual void InitializeZombie(ChSystem* system = 0) override { GetVehicle()->InitializeZombie(system); }

    /// Synchronoize this agents zombie with the rest of the simulation.
    /// Updates agent based on specified message.

    ///@brief Synchronize the position and orientation of this vehicle with other ranks.
    /// Simply calls the same method on the underlying vehicle.
    ///
    ///@param message the received message that describes the position and orientation.
    virtual void SynchronizeZombie(SynMessage* message) override { GetVehicle()->SynchronizeZombie(message); }

    /// Advance the state of this vehicle agent until agent time syncs with passed time.

    ///@brief Advance the state of this agent until agent time syncs with passed time.
    ///
    ///@param time_of_next_sync time at which this agent should stop advancing
    virtual void Advance(double time_of_next_sync) override;

    ///@brief Process an incoming message. Will pass the message directly to this agent's brain.
    ///
    ///@param msg the received message to be processed
    virtual void ProcessMessage(SynMessage* msg) override;

    // --------------------------------------------------------------------------------------------------------------

    /// Get this agent's vehicle
    virtual std::shared_ptr<SynVehicle> GetVehicle() = 0;

    /// Get/Set the underlying terrain
    std::shared_ptr<SynTerrain> GetTerrain() { return m_terrain; }
    void SetTerrain(std::shared_ptr<SynTerrain> terrain) { m_terrain = terrain; }

    /// Get/Set the underlying brain for this agent
    std::shared_ptr<SynVehicleBrain> GetBrain() { return m_brain; }
    void SetBrain(std::shared_ptr<SynVehicleBrain> brain) { m_brain = brain; }

    /// Helper method to get the Chrono driver from the brain
    std::shared_ptr<vehicle::ChDriver> GetDriver() { return m_brain->GetDriver(); }

    /// Helper method to get the ChVehicle from the SynVehicle
    vehicle::ChVehicle& GetChVehicle() { return GetVehicle()->GetVehicle(); }

  protected:
    ///@brief Parse a vehicle agent json specification file
    ///
    ///@param filename the json specification file
    ///@return rapidjson::Document the parsed rapidjson document
    rapidjson::Document ParseVehicleAgentFileJSON(const std::string& filename);

    ///@brief Construct a VehicleAgent from the document
    ///
    ///@param d a rapidjson document that contains the required elements
    void VehicleAgentFromJSON(rapidjson::Document& d);

    std::shared_ptr<SynTerrain> m_terrain;     ///< handle to this agent's terrain
    std::shared_ptr<SynVehicleBrain> m_brain;  ///< handle to this agent's brain
};

/// @} synchrono_agent

}  // namespace synchrono
}  // namespace chrono

#endif
