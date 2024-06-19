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

#include "chrono/physics/ChSystemNSC.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"

#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynTrackedVehicleAgent.h"

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
std::shared_ptr<SynMessage> msg_sent;

// Global var to transfer chrono_manager
SynChronoManager* syn_manager_ptr;

// Define our own main here to handle the MPI setup
int main(int argc, char* argv[]) {
    // Let google strip their cli arguments
    ::testing::InitGoogleTest(&argc, argv);

    // Create the MPI communicator and the manager
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    rank = communicator->GetRank();
    num_ranks = communicator->GetNumRanks();
    SynChronoManager syn_manager(rank, num_ranks, communicator);

    // Chrono system
    ChSystemNSC system;
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    system.GetSolver()->AsIterative()->SetMaxIterations(150);
    system.SetMaxPenetrationRecoverySpeed(4.0);

    int rank_group = rank % 3;

    // Add agents
    switch (rank_group) {
        case 0:
            agent_1 = chrono_types::make_shared<SynWheeledVehicleAgent>();
            syn_manager.AddAgent(agent_1);
            break;
        case 1:
            agent_2 = chrono_types::make_shared<SynTrackedVehicleAgent>();
            syn_manager.AddAgent(agent_2);
            break;
        case 2:
            agent_3 = chrono_types::make_shared<SynEnvironmentAgent>(&system);
            syn_manager.AddAgent(agent_3);
            break;
        default:
            throw std::invalid_argument("Improper enumeration of integers modulo 3");
    }

    // Generate and sync msg for every agent
    syn_manager.Initialize(&system);
    syn_manager_ptr = &syn_manager;

    // Store the current msg buffer
    SynMessageList messages;
    switch (rank_group) {
        case 0:
            agent_1->GatherMessages(messages);
            break;
        case 1:
            agent_2->GatherMessages(messages);
            break;
        case 2:
            agent_3->GatherMessages(messages);
            break;
        default:
            throw std::invalid_argument("Improper enumeration of integers modulo 3");
    }
    msg_sent = messages[0];

    ::testing::TestEventListeners& listeners = ::testing::UnitTest::GetInstance()->listeners();
    if (rank != 0) {
        delete listeners.Release(listeners.default_result_printer());
    }

    // Each rank will be running each test
    return RUN_ALL_TESTS();
}

TEST(SynChrono, SynChronoInit) {
    // Retrieve mpi_manager after sync
    auto check_agent_list = syn_manager_ptr->GetAgents();
    auto check_zombie_list = syn_manager_ptr->GetZombies();

    // Agent list size after sync
    int check_agent_size = static_cast<int>(check_agent_list.size());
    int check_zombie_size = static_cast<int>(check_zombie_list.size());

    // Var to store msg on the current rank after mpi sync
    SynMessageList messages;
    check_agent_list[AgentKey(rank, 1)]->GatherMessages(messages);
    auto msg_recvd = messages[0];

    // Check whether agent size is num_ranks
    EXPECT_EQ(check_agent_size + check_zombie_size, num_ranks);

    // Check whether the msg in the agent on the curr rank is intact
    EXPECT_EQ(msg_recvd, msg_sent);
}