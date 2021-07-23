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

#include <mpi.h>

#include "chrono_vehicle/cosim/ChVehicleCosimBaseNode.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

const double ChVehicleCosimBaseNode::m_gacc = -9.81;

ChVehicleCosimBaseNode::ChVehicleCosimBaseNode(const std::string& name)
    : m_name(name),
      m_step_size(1e-4),
      m_cum_sim_time(0),
      m_verbose(true),
      m_num_mbs_nodes(0),
      m_num_terrain_nodes(0),
      m_num_tire_nodes(0) {}

void ChVehicleCosimBaseNode::Initialize() {
    unsigned int send_data[] = {0, 0, 0};
    switch (GetNodeType()) {
        case NodeType::MBS:
            send_data[0]++;
            break;
        case NodeType::TERRAIN:
            send_data[1]++;
            break;
        case NodeType::TIRE:
            send_data[2]++;
            break;
    }

    unsigned int recv_data[3];
    MPI_Allreduce(send_data, recv_data, 3, MPI_UNSIGNED, MPI_SUM, MPI_COMM_WORLD);
    m_num_mbs_nodes = recv_data[0];
    m_num_terrain_nodes = recv_data[1];
    m_num_tire_nodes = recv_data[2];

    if (m_verbose) {
        cout << "[" << m_name << "]"                           //
             << "  MBS nodes: " << m_num_mbs_nodes             //
             << "  TERRAIN nodes: " << m_num_terrain_nodes     //
             << "  TIRE nodes: " << m_num_tire_nodes << endl;  //
    }
}

void ChVehicleCosimBaseNode::SetOutDir(const std::string& dir_name, const std::string& suffix) {
    m_out_dir = dir_name;
    m_node_out_dir = dir_name + "/" + m_name + suffix;

    if (!filesystem::create_directory(filesystem::path(m_node_out_dir))) {
        std::cout << "Error creating directory " << m_node_out_dir << std::endl;
        return;
    }

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

}  // end namespace vehicle
}  // end namespace chrono
