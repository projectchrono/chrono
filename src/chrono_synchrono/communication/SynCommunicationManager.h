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
// Base class for any class that could manage communication in a Chrono
// simulation (e.g. DDS or MPI). Common tasks are adding agents, advancing the
// physics and synchronizing state data among all participants (ranks in MPI)
//
// =============================================================================

#ifndef SYN_COMMUNICATION_MANAGER_H
#define SYN_COMMUNICATION_MANAGER_H

#include <map>

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/agent/SynAgent.h"

#include "chrono_synchrono/flatbuffer/SynFlatBuffersManager.h"
#include "chrono_synchrono/flatbuffer/message/SynMessage.h"
#include "chrono_synchrono/flatbuffer/message/SynMessageFactory.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication
/// @{

/// Base class for anyone that could manage state synchronization in a chrono simulation
class SYN_API SynCommunicationManager {
  public:
    /// @brief Constructs a new SynCommunicationManager object
    SynCommunicationManager();

    /// @brief Constructs a new SynCommunicationManager object with a starting message length
    SynCommunicationManager(int msg_length);

    /// @brief Destory the SynCommunicationManager object
    ~SynCommunicationManager();

    /// @brief Add the specified agent to this manager's list of agents
    ///
    /// @param agent Handle to the agent that will be added
    virtual bool AddAgent(std::shared_ptr<SynAgent> agent);

    /// @brief Add the specified agent to this manager's list of agents at the provided rank
    ///
    /// @param agent Handle to the agent that will be added
    /// @param rank Rank that manages this agent
    virtual bool AddAgent(std::shared_ptr<SynAgent> agent, int rank);

    /// @brief Initialize all agents.
    ///
    /// @return boolean indicated whether initialization function was successful
    virtual bool Initialize();

    /// @brief Advance the state of the agent simulated by this rank to the specified timestep
    ///
    /// @param time_to_next_sync Timestep to advance the agent's ChSystem to
    virtual void Advance(double time_to_next_sync);

    /// @brief Synchronize all zombie agents within each ranks environment
    virtual void Synchronize() = 0;

    /// @brief Blocks simulation from proceeding until all ranks have reached this method call
    virtual void Barrier() = 0;

    /// @brief Adds all messages to the flatbuffer manager and finishes it size-prefixed (if desired)
    ///
    /// @param size_prefixed finish the buffer with the size infront of it
    virtual void GenerateMessages() final;

    /// @brief Adds all info messages to the flatbuffer manager and finishes it size-prefixed
    virtual void GenerateAgentDescriptionMessage() final;

    ///@brief Processes all of the messages received.
    ///
    ///@param data a pointer to the flatbuffer buffer
    virtual void ProcessMessage(const uint8_t* data);

    /// @brief Processes all messages inside a single buffer. Updates zombie agents and allows agents additional
    /// processing.
    ///
    /// @param buffer A buffer that contains messages received from a particular rank. e.g. state data + other messages
    virtual void ProcessMessageBuffer(const SynFlatBuffers::Buffer* buffer) final;

    /// @brief Processes all messages inside a single buffer. Add agents from their description messages
    ///
    /// @param buffer A buffer that contains messages received from a particular rank. e.g. state data + other messages
    virtual std::shared_ptr<SynAgent> AgentFromDescription(const SynFlatBuffers::Buffer* buffer) final;

    // --------------------------------------------------------------------------------------------------------------

    /// @brief Get agent at specified rank
    std::shared_ptr<SynAgent> GetAgent(int rank) { return m_agent_list[rank]; }

    /// @brief Get rank of current manager
    virtual int GetRank() final { return m_rank; }

    /// @brief Get the number of ranks in the simulation
    virtual int GetNumRanks() final { return m_num_ranks; }

    /// @brief Get the agent list
    virtual std::map<int, std::shared_ptr<SynAgent>> GetAgentList() final { return m_agent_list; }

    ///@brief Is the simulation still running?
    bool IsOk() { return m_ok && m_num_advances * m_heartbeat < m_end_time; }

    /// Get/Set the heartbeat
    double GetHeartbeat() { return m_heartbeat; }
    void SetHeartbeat(double heartbeat) { m_heartbeat = heartbeat; }

    /// Get/Set the end time
    double GetEndTime() { return m_end_time; }
    void SetEndTime(double end_time) { m_end_time = end_time; }

  protected:
    bool m_ok;  ///< Is everything ok?

    // Time variables
    double m_heartbeat;
    double m_end_time;

    int m_num_advances;      ///< keeps track of the number of updates that have occured so far
    bool m_is_synchronized;  ///< keeps track of the whether ranks are time synchronized

    int m_rank;       ///< rank which this manager controls
    int m_num_ranks;  ///< total number of ranks in this simulation

    bool m_initialized;  ///< has the manager been initialized

    std::shared_ptr<SynAgent> m_agent;  ///< handle to this rank's agent

    std::map<int, std::shared_ptr<SynAgent>> m_agent_list;   ///< id to agent map on this rank
    std::map<int, std::vector<SynMessage*>> m_message_list;  ///< received messages mapped to each rank

    SynFlatBuffersManager m_flatbuffers_manager;  ///< flatbuffer manager for this rank
};

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif
