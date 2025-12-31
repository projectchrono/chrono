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
// Authors: Jay Taves
// =============================================================================
//
// Unit test for SynChrono MPI code
//
// =============================================================================

#include <numeric>

#include "gtest/gtest.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"

#include "chrono_synchrono/utils/SynDataPath.h"

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"

using namespace chrono;
using namespace synchrono;

int rank;
int num_ranks;

// Define our own main here to handle the MPI setup
int main(int argc, char* argv[]) {
    // Let google strip their cli arguments
    ::testing::InitGoogleTest(&argc, argv);

    // Create the MPI communicator and the manager
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    rank = communicator->GetRank();
    num_ranks = communicator->GetNumRanks();
    SynChronoManager syn_manager(rank, num_ranks, communicator);

    ::testing::TestEventListeners& listeners = ::testing::UnitTest::GetInstance()->listeners();
    if (rank != 0) {
        delete listeners.Release(listeners.default_result_printer());
    }

    // Each rank will be running each test
    return RUN_ALL_TESTS();
}

TEST(SynChrono, SynChronoInit) {
    int* msg_lengths = new int[num_ranks];
    int* msg_displs = new int[num_ranks];

    // Just a meaningless message length that we will fill with data
    int msg_length = 10 + num_ranks - rank;

    // Determine how much we stuff we get from other ranks
    MPI_Allgather(&msg_length, 1, MPI_INT,  // Sending args
                  msg_lengths, 1, MPI_INT,  // Receiving args
                  MPI_COMM_WORLD);

    int total_length = 0;
    std::vector<int> all_data;
    std::vector<int> my_data;

    // Fill the data we will send
    for (int i = 0; i < msg_length; i++)
        my_data.push_back(rank);

    // Compute offsets for our receiving buffer
    // In C++17 this could just be an exclusive scan from std::
    // Didn't use std::partial_sum since we want m_total_length computed
    // m_msg_displs is needed by MPI_Gatherv
    for (int i = 0; i < num_ranks; i++) {
        msg_displs[i] = total_length;
        total_length += msg_lengths[i];
    }

    // Need resize rather than reserve so that MPI can just copy into the buffer
    all_data.resize(total_length);

    MPI_Allgatherv(my_data.data(), msg_length, MPI_INT,                // Sending args
                   all_data.data(), msg_lengths, msg_displs, MPI_INT,  // Receiving args
                   MPI_COMM_WORLD);

    int sum = std::accumulate(all_data.begin(), all_data.end(), 0);

    // Σ rank * (10 + num_ranks - rank)     from 0 -> num_ranks - 1
    int check = (num_ranks - 1) * num_ranks * (num_ranks + 31) / 6;

    MPI_Barrier(MPI_COMM_WORLD);

    ASSERT_EQ(sum, check);

    delete[] msg_lengths;
    delete[] msg_displs;
}