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
// interfaecs either a rank with another rank, a rank with an external process
// or really anything that relies on communication over some network interface.
//
// This class is implemented as a very generic abstract handler that holds and
// defines common functions and variables used by all communicators.
//
// =============================================================================

#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"

namespace chrono {
namespace synchrono {

SynMPICommunicator::SynMPICommunicator(int argc, char* argv[]) {
    // mpi initialization
    MPI_Init(&argc, &argv);
    // set rank
    MPI_Comm_rank(MPI_COMM_WORLD, &m_rank);
    // set number of ranks in this simulation
    MPI_Comm_size(MPI_COMM_WORLD, &m_num_ranks);

    Barrier();

    m_msg_lengths = new int[m_num_ranks];
    m_msg_displs = new int[m_num_ranks];
}

SynMPICommunicator::~SynMPICommunicator() {
    delete[] m_msg_lengths;
    delete[] m_msg_displs;

    MPI_Finalize();
}

void SynMPICommunicator::Synchronize() {
    m_flatbuffers_manager.Finish();

    int msg_length = m_flatbuffers_manager.GetSize();

    // Get the length of message from each agent
    MPI_Allgather(&msg_length, 1, MPI_INT,    // Sending pointer, length, type
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

    // if (m_rank == 0)
    //     std::cout << m_rank << " message length: " << m_total_length << std::endl;

    m_all_data.reserve(m_total_length);

    MPI_Allgatherv(m_flatbuffers_manager.GetBufferPointer(), msg_length, MPI_BYTE,  // Sending pointer, length, type
                   m_all_data.data(), m_msg_lengths, m_msg_displs,
                   MPI_BYTE,  // Receiving pointer, lengths, displacements, type
                   MPI_COMM_WORLD);

    m_flatbuffers_manager.Reset();
}

SynMessageList& SynMPICommunicator::GetMessages() {
    for (int i = 0; i < m_num_ranks; i++) {
        if (i != m_rank) {
            std::vector<uint8_t> data = std::vector<uint8_t>(m_all_data.data() + m_msg_displs[i],
                                                             m_all_data.data() + m_msg_displs[i] + m_msg_lengths[i]);
            m_flatbuffers_manager.ProcessBuffer(data, m_incoming_messages);
        }
    }

    return m_incoming_messages;
}

}  // namespace synchrono
}  // namespace chrono