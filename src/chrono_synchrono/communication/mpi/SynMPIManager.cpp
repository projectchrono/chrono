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
// Authors: Aaron Young, Jay Taves
// =============================================================================
//
// Concrete communication class that manages the state synchronization between
// various SynChrono entities. Uses MPI gatherAllv calls to send state messages
// between all ranks, representing each agent my a single MPI rank.
//
// =============================================================================

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono/core/ChLog.h"

namespace chrono {
namespace synchrono {

const SynMPIConfig MPI_CONFIG_DEFAULT;

SynMPIManager::SynMPIManager(int argc, char* argv[], SynMPIConfig config) : m_config(config) {
    // mpi initialization
    MPI_Init(&argc, &argv);
    // set rank
    MPI_Comm_rank(MPI_COMM_WORLD, &m_rank);
    // set number of ranks in this simulation
    MPI_Comm_size(MPI_COMM_WORLD, &m_num_ranks);

    SynMPIManager::Barrier();

    m_msg_lengths = new int[m_num_ranks];
    m_msg_displs = new int[m_num_ranks];

    // Round up to the nearest multiple of uoffset_t
    auto mult = sizeof(flatbuffers::uoffset_t);
    m_msg_length = ((m_config.max_msg_length + mult - 1) / mult) * mult;

    switch (m_config.memory_mode) {
        case SynMPIMemoryMode::PREALLOCATED: {
            // Preallocate the data arrays to hold the maximum length allowed
            m_all_data.reserve(m_msg_length * m_num_ranks);

            for (int i = 0; i < m_num_ranks; i++) {
                m_msg_lengths[i] = m_msg_length;
                m_msg_displs[i] = m_msg_length * i;
            }

            break;
        }
        case SynMPIMemoryMode::PREALLOCATED_WITH_REALLOC: {
            // Preallocate the data arrays to hold the maximum length allowed
            // May need to reallocate later
            m_all_data.reserve(m_msg_length * m_num_ranks);

            break;
        }
        case SynMPIMemoryMode::DYNAMIC_RESERVE: {
            // Do nothing since we will reallocate automatically
        }
        default:
            break;
    }
}

// Destructor
SynMPIManager::~SynMPIManager() {
    delete m_msg_lengths;
    delete m_msg_displs;

    MPI_Finalize();
}

bool SynMPIManager::Initialize() {
    // Add all of the messages to the FlatBufferManager
    GenerateAgentDescriptionMessage();

    // Send the messages out to each rank and add each new agent accordingly
    Synchronize();
    Update();

    // Wait for all the ranks to catch up
    Barrier();

    return SynCommunicationManager::Initialize();
}

void SynMPIManager::Synchronize() {
    if (m_initialized)
        // Generate the messages that will be sent to be used to synchronize with other ranks
        GenerateMessages();

    // Determine the outgoing message size
    switch (m_config.memory_mode) {
        case SynMPIMemoryMode::PREALLOCATED: {
            // Do nothing since preallocation happens at the very beginning.
            int size = m_flatbuffers_manager.GetSize();
            if (size > m_msg_length)
                GetLog() << "Rank " << m_rank << " message size: " << size
                         << " > maximum message size: " << m_msg_length << "\n";

            break;
        }
        case SynMPIMemoryMode::PREALLOCATED_WITH_REALLOC: {
            // Check to see if a realloc is necessary
            // Only necessary if message size is greater than current buffer size
            auto size = m_flatbuffers_manager.GetSize();
            if (size > m_msg_length)
                m_msg_length = size;

            break;
        }
        case SynMPIMemoryMode::DYNAMIC_RESERVE: {
            // Always reallocate based on the buffer size
            m_msg_length = m_flatbuffers_manager.GetSize();
            break;
        }
        default:
            break;
    }

    if (m_config.memory_mode != SynMPIMemoryMode::PREALLOCATED) {
        // Get the incoming/send the outgoing message size since we may need to realloc
        // Get the length of message from each agent
        // All ranks will send their incoming buffer sizes to each other rank
        MPI_Allgather(&m_msg_length, 1, MPI_INT,  // Sending pointer, length, type
                      m_msg_lengths, 1, MPI_INT,  // Receiving pointer, length, type
                      MPI_COMM_WORLD);            // Receiving rank and world

        m_total_length = 0;

        // In C++17 this could just be an exclusive scan from std::
        // Didn't use std::partial_sum since we want m_total_length computed
        // m_msg_displs is needed by MPI_Gatherv
        for (int i = 0; i < m_num_ranks; i++) {
            m_msg_displs[i] = m_total_length;
            m_total_length += m_msg_lengths[i];
        }

        if (m_total_length > m_all_data.capacity() || m_config.memory_mode == SynMPIMemoryMode::DYNAMIC_RESERVE)
            // If necessary, reserve enough space for the incoming data
            m_all_data.reserve(m_total_length);
    }

    // Pointers to the serialized flatbuffer and all rank data for convenience
    uint8_t* rank_data = m_flatbuffers_manager.GetBufferPointer();
    uint8_t* all_data = m_all_data.data();

    if (m_config.memory_mode != SynMPIMemoryMode::DYNAMIC_RESERVE)
        // Change the size prefix to contain the empty space after the actual message
        // This will occur for the preallocated memory modes since the buffer is greater than or equal to the actual
        // message size
        ((int*)rank_data)[0] = m_msg_length - sizeof(flatbuffers::uoffset_t);

    // DESIGN DECISION:
    // Always use MPI___v(...) for simplicity sake. Not actually sure about the performance implications
    MPI_Allgatherv(rank_data, m_msg_length, MPI_BYTE,                // Sending ptr, length, type
                   all_data, m_msg_lengths, m_msg_displs, MPI_BYTE,  // Receiving ptr, lengths, displacements, type
                   MPI_COMM_WORLD);                                  // world
}

void SynMPIManager::Update() {
    ProcessMessage(m_all_data.data());
}

}  // namespace synchrono
}  // namespace chrono
