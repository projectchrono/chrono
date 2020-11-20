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
// simulation (e.g. MPI). Common tasks are adding agents, advancing the
// physics and synchronizing state data among all participants (ranks in MPI)
//
// =============================================================================

#include "chrono_synchrono/communication/SynCommunicationManager.h"
#include "chrono_synchrono/agent/SynAgentFactory.h"

#include "chrono/core/ChLog.h"

namespace chrono {
namespace synchrono {

SynCommunicationManager::SynCommunicationManager()
    : m_initialized(false),
      m_num_advances(0),
      m_is_synchronized(true),
      m_ok(true),
      m_heartbeat(1e-2),
      m_end_time(1000) {}

SynCommunicationManager::SynCommunicationManager(int msg_length)
    : m_flatbuffers_manager(msg_length),
      m_initialized(false),
      m_num_advances(0),
      m_is_synchronized(true),
      m_ok(true),
      m_heartbeat(1e-2),
      m_end_time(1000) {}

SynCommunicationManager::~SynCommunicationManager() {}

// Add the agent
// Grab the rank directly from the agent
bool SynCommunicationManager::AddAgent(std::shared_ptr<SynAgent> agent) {
    int rank = agent->GetRank();
    return AddAgent(agent, rank);
}

// Set the agent at the specified rank
bool SynCommunicationManager::AddAgent(std::shared_ptr<SynAgent> agent, int rank) {
    if (rank == m_rank) {
        auto it = m_agent_list.find(m_rank);
        if (it != m_agent_list.end()) {
            GetLog() << "SynCommunicationManager::AddAgent: Each agent should have a unique rank."
                     << "\n";
            return false;
        }
    }

    m_agent_list[rank] = agent;

    if (rank == m_rank)
        // Grab this rank's agent
        m_agent = agent;

    return true;
}

// Initialize agents and their zombies
bool SynCommunicationManager::Initialize() {
    // Check if this rank has an agent
    if (m_agent_list.find(m_rank) == m_agent_list.end()) {
        GetLog() << "Agent list does not contain an agent for this rank."
                 << "\n";
        return false;
    }

    // Initialize the zombies
    for (auto& pair : m_agent_list) {
        if (pair.first != m_rank && m_agent->GetSystem()) {
            pair.second->InitializeZombie(m_agent->GetSystem());
        }
        m_message_list[pair.first] = std::vector<SynMessage*>();
    }

    m_initialized = true;

    return true;
}

// Gather updated state info and msgs to send for agents on this rank
void SynCommunicationManager::Advance(double time_to_next_sync) {
    // Reset FlatBuffer buffer
    m_flatbuffers_manager.Reset();

    for (auto& p1 : m_agent_list) {
        // For readability
        int rank = p1.first;
        std::shared_ptr<SynAgent> temp_agent = p1.second;
        std::vector<SynMessage*>& msgs = m_message_list[rank];

        if (rank == m_rank)
            continue;

        // If no messages were received on this rank, synchronize anyways
        if (msgs.size() == 0)
            temp_agent->SynchronizeZombie(nullptr);

        for (auto msg : msgs) {
            // Only synchronize zombie if  agent has a ChSystem
            if (m_agent->GetSystem())
                temp_agent->SynchronizeZombie(msg);

            // Agent can take additional actions with the msg
            m_agent->ProcessMessage(msg);

            // Clean up
            delete msg;
        }

        // Clear list
        msgs.clear();
    }

    if (!m_is_synchronized)
        return;

    // currently one agent per rank
    m_agent->Advance(time_to_next_sync);
    m_num_advances++;
}

// Collect all of the messages that need to be sent by this rank e.g. Vehicle state data + terrain state data + V2V
// messages
void SynCommunicationManager::GenerateMessages() {
    std::vector<SynMessage*> messages;
    m_agent->GenerateMessagesToSend(messages);
    for (auto msg : messages) {
        m_flatbuffers_manager.AddMessage(msg);
        delete msg;
    }

    // Complete the FlatBuffer buffer with size included as the first 4 bytes
    m_flatbuffers_manager.FinishSizePrefixed();
}

// Collect all of the info messages that need to be sent by this rank e.g. Vehicle state data + terrain state data + V2V
// messages
void SynCommunicationManager::GenerateAgentDescriptionMessage() {
    // Create the info message and add it to the flatbuffer manager
    auto msg = m_agent->GetMessage();

    if (!msg)
        return;

    auto description = msg->MessageFromDescription(m_flatbuffers_manager.GetBuilder());
    m_flatbuffers_manager.AddMessage(description);

    // Complete the FlatBuffer buffer with size included as the first 4 bytes
    m_flatbuffers_manager.FinishSizePrefixed();
}

void SynCommunicationManager::ProcessMessage(const uint8_t* data) {
    for (int i = 0; i < m_num_ranks; i++) {
        // Get a buffer pointer from the raw bytes of the flatbuffer message
        auto buffer = flatbuffers::GetSizePrefixedRoot<SynFlatBuffers::Buffer>(data);

        ProcessMessageBuffer(buffer);

        // Move to the next agent's portion of the buffer
        data += flatbuffers::GetPrefixedSize(data) + sizeof(flatbuffers::uoffset_t);
    }
}

// Process all messages contained in a buffer of messages. e.g. Update Zombie vehicles, update terrain data, ...
void SynCommunicationManager::ProcessMessageBuffer(const SynFlatBuffers::Buffer* buffer) {
    if (!m_initialized) {
        auto agent = AgentFromDescription(buffer);
        if (agent)
            AddAgent(agent);

        return;
    }

    // Simulation will be assumed to be synchronized
    m_is_synchronized = true;

    // This keeps going until it hits the end of the data contained in agent i's portion of the buffer
    for (auto message : (*buffer->buffer())) {
        SynMessage* msg = SynMessageFactory::GenerateMessage(message);
        if (!msg)
            continue;

        int rank = msg->GetRank();

        // Ignore message if sent by this rank or this rank doesn't have an agent
        if (m_rank == rank)
            continue;

        // Verify we are still in sync
        m_is_synchronized = msg->GetState()->time < m_heartbeat * (m_num_advances - 1) ? false : m_is_synchronized;

        // Store messages to be processed next
        m_message_list[rank].push_back(msg);
    }
}

std::shared_ptr<SynAgent> SynCommunicationManager::AgentFromDescription(const SynFlatBuffers::Buffer* buffer) {
    std::shared_ptr<SynAgent> agent = nullptr;

    auto message = buffer->buffer()->Get(0);
    SynMessage* msg = SynMessageFactory::GenerateMessage(message);
    if (!msg)
        return agent;

    if (!m_initialized && m_agent_list.find(msg->GetRank()) == m_agent_list.end()) {
        // TODO: Safer cast
        auto agent_msg = (SynAgentMessage*)msg;

        // Create and add the agent
        agent = SynAgentFactory::CreateAgent(agent_msg);

        // Clean up
        delete msg;
    }

    return agent;
}

}  // namespace synchrono
}  // namespace chrono