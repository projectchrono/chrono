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

#ifndef SYN_MPI_MANAGER_H
#define SYN_MPI_MANAGER_H

#include "chrono_synchrono/SynApi.h"

#include "chrono_synchrono/communication/SynCommunicationManager.h"

#include <mpi.h>

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication_mpi
/// @{

///@brief Memory mode for the MPIManager
///
/// PREALLOCATED: Least allocations. Largest memory footprint relative to message size. May result in inadequate
///               space for incoming/outgoing messages!
/// PREALLOCATED_WITH_REALLOC: If used well, better memory footprint than PREALLOCATED at the cost of more
///                            allocations. Ensures always enough space for incoming/outgoing messages.
/// DYNAMIC_RESERVE: Least memory footprint (always exactly enough space) with highest number of allocations.
enum class SynMPIMemoryMode {
    PREALLOCATED,               ///< Preallocated at the beginning of the simulation
    PREALLOCATED_WITH_REALLOC,  ///< Preallocated at the beginning of the simulation but reallocated if new message
                                ///< size is larger than current buffer size
    DYNAMIC_RESERVE             ///< Buffer is reallocated every time a message is received
};

struct SYN_API SynMPIConfig {
    SynMPIMemoryMode memory_mode = SynMPIMemoryMode::PREALLOCATED_WITH_REALLOC;

    int max_msg_length = 1024;  ///< Maximum message size. Sets to reasonable value to start.
};

SYN_API extern const SynMPIConfig MPI_CONFIG_DEFAULT;

/// Concrete class using MPI AllGatherV calls to manage state synchronization
class SYN_API SynMPIManager : public SynCommunicationManager {
  public:
    SynMPIManager(int argc, char* argv[], SynMPIConfig config = MPI_CONFIG_DEFAULT);
    ~SynMPIManager();

    virtual void Exit() {
        MPI_Finalize();
        exit(0);
    }

    /// @brief Generate agent description messages and synchronize them to all agents
    ///
    /// @return boolean indicated whether initialization function was successful
    virtual bool Initialize() override;

    /// @brief Gather messages from all ranks and synchronize the simulation
    virtual void Synchronize() override;

    /// @brief Update the zombie agents and process messages
    ///
    /// Overriding classes should probably at a minimum call ProcessBufferedMessages. All worlds should be guaranteed to
    /// be synchronized after this function returns.
    virtual void Update();

    /// Wrapper of MPI barrier
    virtual void Barrier() override { MPI_Barrier(MPI_COMM_WORLD); }

    /// Get the MPI Config
    SynMPIConfig& GetConfig() { return m_config; }

  private:
    void InitializeMPI(int argc, char* argv[]);

  protected:
    SynMPIConfig m_config;

    int m_msg_length;    ///< Maximum size of message that can be sent from any agent
    int m_bcast_length;  ///< Size of the buffer that gets broadcast to all agents (num_ranks - 1) * m_msg_length

    int m_total_length;  ///< Current length of messages from all ranks

    int* m_msg_lengths;  ///< Array of message lengths for all ranks
    int* m_msg_displs;   ///< Array of message displacements for all ranks

    std::vector<uint8_t> m_rank_data;
    std::vector<uint8_t> m_all_data;  ///< Buffer for receiving messages from all ranks
};

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono
#endif
