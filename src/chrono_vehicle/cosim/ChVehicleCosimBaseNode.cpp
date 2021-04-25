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

namespace chrono {
namespace vehicle {

const double ChVehicleCosimBaseNode::m_gacc = -9.81;

ChVehicleCosimBaseNode::ChVehicleCosimBaseNode(const std::string& name)
    : m_name(name), m_step_size(1e-4), m_cum_sim_time(0) {}

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

}  // end namespace vehicle
}  // end namespace chrono
