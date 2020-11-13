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
// Authors: Jason Zhou
// =============================================================================
//
// Unit test for SynMessage Initialization
//
// =============================================================================

#include "gtest/gtest.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"

#include "chrono_synchrono/brain/SynACCBrain.h"
#include "chrono_synchrono/brain/SynEnvironmentBrain.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"

using namespace chrono;
using namespace synchrono;

int rank;
int num_ranks;

// Global Variable Declaration
// Three different types of agents will be tested
std::shared_ptr<SynWheeledVehicleAgent> agent_1;
std::shared_ptr<SynTrackedVehicleAgent> agent_2;
std::shared_ptr<SynEnvironmentAgent> agent_3;

// Var stores the msg from the curr rank for comparison
std::shared_ptr<SynAgentMessage> msg_sent;

// Global var to transfer mpi_manager
SynMPIManager* mpi_manager_ptr;

// Define our own main here to handle the MPI setup
int main(int argc, char* argv[]) {
    // Let google strip their cli arguments
    ::testing::InitGoogleTest(&argc, argv);

    // Let MPI handle their cli arguments
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    mpi_manager_ptr = &mpi_manager;
    rank = mpi_manager.GetRank();
    num_ranks = mpi_manager.GetNumRanks();

    // Add agents
    if (rank % 2 == 0) {
        agent_1 = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        mpi_manager.AddAgent(agent_1, rank);
    } else if (rank % 2 == 1) {
        agent_2 = chrono_types::make_shared<SynTrackedVehicleAgent>(rank);
        mpi_manager.AddAgent(agent_2, rank);
    } else if (rank % 3 == 1) {
        agent_3 = chrono_types::make_shared<SynEnvironmentAgent>(rank);
        mpi_manager.AddAgent(agent_3, rank);
    }

    // Generate and sync msg for every agent
    mpi_manager.Initialize();

    // Store the current msg buffer
    if (rank % 2 == 0) {
        msg_sent = agent_1->GetMessage();
    } else if (rank % 2 == 1) {
        msg_sent = agent_2->GetMessage();
    } else if (rank % 3 == 1) {
        msg_sent = agent_3->GetMessage();
    }

    ::testing::TestEventListeners& listeners = ::testing::UnitTest::GetInstance()->listeners();
    if (rank != 0) {
        delete listeners.Release(listeners.default_result_printer());
    }

    // Each rank will be running each test
    return RUN_ALL_TESTS();
}

TEST(SynVehicle, SynVehicleInit) {
    // Retrieve mpi_manager after sync
    std::map<int, std::shared_ptr<SynAgent>> check_agent_list = mpi_manager_ptr->GetAgentList();

    // Agent list size after sync
    int check_agent_size = check_agent_list.size();

    // Var to store msg on the current rank after mpi sync
    std::shared_ptr<SynAgentMessage> msg_rcevd = check_agent_list[rank]->GetMessage();

    // Check whether agent size is num_ranks
    EXPECT_EQ(check_agent_size, num_ranks);

    // Check whether the msg in the agent on the curr rank is intact
    EXPECT_EQ(msg_rcevd, msg_sent);
}