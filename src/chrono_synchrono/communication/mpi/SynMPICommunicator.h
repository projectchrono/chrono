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
// =============================================================================
//
// Class that handles communication across ranks or external entities. A
// communicator is something that passes messages over some protocol and
// interfaces either a rank with another rank, a rank with an external process
// or really anything that relies on communication over some network interface.
//
// This class is implemented as a very generic abstract handler that holds and
// defines common functions and variables used by all communicators.
//
// =============================================================================

#ifndef SYN_MPI_COMMUNICATOR_H
#define SYN_MPI_COMMUNICATOR_H

#include <mpi.h>

#include "chrono_synchrono/communication/SynCommunicator.h"

namespace chrono {
namespace synchrono {

/// @addtogroup synchrono_communication_mpi
/// @{

/// Derived communicator used to establish and facilitate communication between nodes.
/// Uses the Message Passing Interface (MPI) standard
class SYN_API SynMPICommunicator : public SynCommunicator {
  public:
    ///@brief Default constructor
    ///
    SynMPICommunicator(int argc, char** argv);

    ///@brief Destructor
    ///
    virtual ~SynMPICommunicator();

    ///@brief Initialization method typically responsible for establishing a connection.
    /// Although not mandatory, this method should handle initial peer-to-peer communication.
    /// This could mean a simple handshake or an actual exchange of information used during the simulation.
    ///
    virtual void Initialize() override{};

    ///@brief This method is responsible for continuous synchronization steps
    /// This method, depending on it's implementation, could be blocking or non-blocking.
    /// As in, depending on the derived class implementation, this method could use synchronous
    /// or asynchronous method calls to implement a communication interface.
    ///
    /// Synchronize will serialize the underlying message buffer if it hasn't yet
    ///
    virtual void Synchronize() override;

    ///@brief This method is responsible for blocking until an action is received or done.
    /// For example, a process may call Barrier to wait until another process has established
    /// certain classes and initialized certain quantities. This functionality should be implemented
    /// in this method.
    ///
    virtual void Barrier() override { MPI_Barrier(MPI_COMM_WORLD); }

    // -----------------------------------------------------------------------------------------------

    ///@brief Get the messages received by the communicator
    ///
    ///@return SynMessageList the received messages
    virtual SynMessageList& GetMessages() override;

    ///@brief Get the rank for the attached process
    ///
    virtual int GetRank() const { return m_rank; }

    ///@brief Get the number of ranks in the MPI world
    ///
    virtual int GetNumRanks() const { return m_num_ranks; }

    // -----------------------------------------------------------------------------------------------

  private:
    int m_rank;
    int m_num_ranks;

    int m_total_length;

    int* m_msg_lengths;
    int* m_msg_displs;

    std::vector<uint8_t> m_rank_data;
    std::vector<uint8_t> m_all_data;
};

/// @} synchrono_communication

}  // namespace synchrono
}  // namespace chrono

#endif