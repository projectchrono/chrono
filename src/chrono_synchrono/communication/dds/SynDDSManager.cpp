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
// File description
//
// =============================================================================

#include "chrono_synchrono/communication/dds/SynDDSManager.h"

#include "chrono_synchrono/communication/dds/SynDDSParticipant.h"

namespace chrono {
namespace synchrono {

const SynDDSConfig DDS_CONFIG_DEFAULT;

// Constructor
SynDDSManager::SynDDSManager(int rank, int num_ranks, SynDDSConfig config)
    : m_participant(new SynDDSParticipant(std::to_string(rank))),
      m_config(config),
      m_last(std::chrono::high_resolution_clock::now()) {
    m_rank = rank;
    m_num_ranks = num_ranks;
}

// Destructor
SynDDSManager::~SynDDSManager() {
    delete m_participant;
}

bool SynDDSManager::Initialize() {
    m_participant->Initialize();

    // Create a publisher
    // This publisher will be responsible for publishing state information of the agent
    auto publisher = new SynDDSPublisher(m_participant->GetParticipant(), std::to_string(m_rank));
    if (!publisher->Initialize())
        return false;
    m_participant->PushPublisher(publisher);
    m_publisher = publisher;

    // Wait for all participants to be available
    Barrier();

    // Add all of the messages to the FlatBufferManager
    GenerateAgentDescriptionMessage();

    // Publish the agent description messages
    Publish();
    Listen(true);

    // Initialize by adding agents to the communication manager
    bool result = SynCommunicationManager::Initialize();

    if (m_config.mode == SynDDSMode::ASYNCHRONOUS)
        // Publish first state message
        Publish();

    return result;
}

// Synchronize the state of the zombie
// TODO: Send more information than just state
void SynDDSManager::Synchronize() {
    if (!m_participant->IsOk()) {
        m_ok = false;
        return;
    }

    if (m_config.mode == SynDDSMode::SYNCHRONOUS) {
        Publish();
        Listen(true);

        return;
    }

    // auto time = std::chrono::high_resolution_clock::now();
    // auto span = std::chrono::duration_cast<std::chrono::milliseconds>(time - m_last).count();
    // if (span >= 25) {
    // m_last = time;

    bool message_received = Listen();

    if (message_received)
        Publish();
    // }
}

void SynDDSManager::Barrier() {
    m_participant->WaitForMatches(m_num_ranks);
}

void SynDDSManager::Publish() {
    if (m_initialized)
        // Generate the messages that will be sent to be used to synchronize with other ranks
        GenerateMessages();

    SynDDSMessage msg;
    msg.data(m_flatbuffers_manager.ToMessageBuffer());
    msg.rank(m_rank);

    m_publisher->Publish(msg);

    m_flatbuffers_manager.Reset();
}

bool SynDDSManager::Listen(bool synchronous) {
    bool message_received = false;

    for (auto subscriber : m_participant->GetSubscribers()) {
        // Block if in synchronous mode
        while (synchronous && !subscriber->HasData())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        if (subscriber->HasData()) {
            message_received = true;

            // Get a buffer pointer from the raw bytes of the flatbuffer message
            auto data = subscriber->GetData();
            auto buffer = flatbuffers::GetSizePrefixedRoot<SynFlatBuffers::Buffer>(data.data());
            ProcessMessageBuffer(buffer);
            subscriber->PopData();
        }
    }

    return message_received;
}

}  // namespace synchrono
}  // namespace chrono
