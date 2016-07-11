// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
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
// Global settings.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "BaseNode.h"
#include "chrono/core/ChFileutils.h"

const double BaseNode::m_gacc = -9.81;

BaseNode::BaseNode(const std::string& name) : m_name(name), m_step_size(1e-4), m_cum_sim_time(0) {}

void BaseNode::SetOutDir(const std::string& dir_name, const std::string& suffix) {
    m_out_dir = dir_name;
    m_node_out_dir = dir_name + "/" + m_name + suffix;

    if (chrono::ChFileutils::MakeDirectory(m_node_out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << m_node_out_dir << std::endl;
        return;
    }

    m_outf.open(m_node_out_dir + "/results.dat", std::ios::out);
    m_outf.precision(7);
    m_outf << std::scientific;
}
