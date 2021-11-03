// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Base class for a co-simulation node.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Free functions in the cosim namespace
// -----------------------------------------------------------------------------

namespace cosim {

static MPI_Comm terrain_comm = MPI_COMM_NULL;

int InitializeFramework(int num_tires) {
    int world_size;
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    if (world_size < 2 + num_tires) {
        return MPI_ERR_OTHER;
    }

    // Get the group for MPI_COMM_WORLD
    MPI_Group world_group;
    MPI_Comm_group(MPI_COMM_WORLD, &world_group);

    // Set list of excluded ranks (vehicle and tire nodes)
    std::vector<int> excluded;
    excluded.push_back(MBS_NODE_RANK);
    for (int i = 0; i < num_tires; i++)
        excluded.push_back(TIRE_NODE_RANK(i));

    // Create the group of ranks for terrain simulation
    MPI_Group terrain_group;
    MPI_Group_excl(world_group, 1 + num_tires, excluded.data(), &terrain_group);

    // Create and return a communicator from the terrain group
    MPI_Comm_create(MPI_COMM_WORLD, terrain_group, &terrain_comm);

    return MPI_SUCCESS;
}

bool IsFrameworkInitialized() {
    return terrain_comm != MPI_COMM_NULL;
}

MPI_Comm GetTerrainIntracommunicator() {
    return terrain_comm;
}

}  // end namespace cosim

// -----------------------------------------------------------------------------

const double ChVehicleCosimBaseNode::m_gacc = -9.81;

ChVehicleCosimBaseNode::ChVehicleCosimBaseNode(const std::string& name)
    : m_name(name),
      m_step_size(1e-4),
      m_cum_sim_time(0),
      m_verbose(true),
      m_num_mbs_nodes(0),
      m_num_terrain_nodes(0),
      m_num_tire_nodes(0),
      m_rank(-1) {
    MPI_Comm_rank(MPI_COMM_WORLD, &m_rank);
}

void ChVehicleCosimBaseNode::Initialize() {
    int size;
    MPI_Comm_size(MPI_COMM_WORLD, &size);

    // Gather node types from all ranks
    int type = -1;
    switch (GetNodeType()) {
        case NodeType::MBS:
            type = 0;
            break;
        case NodeType::TERRAIN:
            type = 1;
            break;
        case NodeType::TIRE:
            type = 2;
            break;
    }
    int* type_all = new int[size];
    MPI_Allgather(&type, 1, MPI_INT, type_all, 1, MPI_INT, MPI_COMM_WORLD);

    // Calculate number of different node types
    for (int i = 0; i < size; i++) {
        switch (type_all[i]) {
            case 0:
                m_num_mbs_nodes++;
                break;
            case 1:
                m_num_terrain_nodes++;
                break;
            case 2:
                m_num_tire_nodes++;
                break;
        }
    }

    if (m_verbose && m_rank == 0) {
        cout << "Num nodes: " << size                        //
             << "  MBS nodes: " << m_num_mbs_nodes             //
             << "  TERRAIN nodes: " << m_num_terrain_nodes     //
             << "  TIRE nodes: " << m_num_tire_nodes << endl;  //
    }

    // Error checks
    bool err = false;

    if (m_num_mbs_nodes != 1) {
        if (m_rank == 0)
            cerr << "ERROR: More than one MBS node." << endl;
        err = true;
    }

    if (type_all[MBS_NODE_RANK] != 0) {
        if (m_rank == 0)
            cerr << "Error: rank " << MBS_NODE_RANK << " is not running an MBS node." << endl;
        err = true;
    }

    if (type_all[TERRAIN_NODE_RANK] != 1) {
        if (m_rank == 0)
            cerr << "Error: rank " << TERRAIN_NODE_RANK << " is not running a TERRAIN node." << endl;
        err = true;
    }

    for (unsigned int i = 0; i < m_num_tire_nodes; i++) {
        if (type_all[TIRE_NODE_RANK(i)] != 2) {
            if (m_rank == 0)
                cerr << "Error: rank " << TIRE_NODE_RANK(i) << " is not running a TIRE node." << endl;
            err = true;
        }
    }

    if (err) {
        MPI_Abort(MPI_COMM_WORLD, 1);
    }
}

void ChVehicleCosimBaseNode::SetOutDir(const std::string& dir_name, const std::string& suffix) {
    m_out_dir = dir_name;
    m_node_out_dir = dir_name + "/" + m_name + suffix;

    // Create node-specific output directory
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir))) {
        std::cout << "Error creating directory " << m_node_out_dir << std::endl;
        return;
    }

    // Create subdirectories for simulation and visualization outputs
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/simulation"))) {
        std::cout << "Error creating directory " << m_node_out_dir + "/simulation" << std::endl;
        return;
    }
    if (!filesystem::create_directory(filesystem::path(m_node_out_dir + "/visualization"))) {
        std::cout << "Error creating directory " << m_node_out_dir + "/visualization" << std::endl;
        return;
    }

    // Create results output file
    m_outf.open(m_node_out_dir + "/results.dat", std::ios::out);
    m_outf.precision(7);
    m_outf << std::scientific;
}

std::string ChVehicleCosimBaseNode::OutputFilename(const std::string& dir,
                                                   const std::string& root,
                                                   const std::string& ext,
                                                   int frame,
                                                   int frame_digits) {
    std::string format = "%s/%s_%0" + std::to_string(frame_digits) + "d.%s";

    size_t buf_size = dir.size() + root.size() + ext.size() + 3 + frame_digits + 1;
    auto buf = new char[buf_size];
    sprintf(buf, format.c_str(), dir.c_str(), root.c_str(), frame, ext.c_str());
    std::string filename(buf);
    delete[] buf;

    return filename;
}

bool ChVehicleCosimBaseNode::IsCosimNode() const {
    if (m_num_terrain_nodes == 1)
        return true;
    if (m_rank == TERRAIN_NODE_RANK)
        return true;
    return GetNodeType() != NodeType::TERRAIN;
}

std::string ChVehicleCosimBaseNode::GetNodeTypeString() const {
    switch (GetNodeType()) {
        case NodeType::MBS:
            return "MBS";
        case NodeType::TIRE:
            return "Tire";
        case NodeType::TERRAIN:
            return "Terrain";
        default:
            return "Unknown";
    }
}

}  // end namespace vehicle
}  // end namespace chrono
