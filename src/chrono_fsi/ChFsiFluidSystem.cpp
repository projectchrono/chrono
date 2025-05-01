// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Base class for an FSI-aware fluid solver
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fsi/ChFsiFluidSystem.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

ChFsiFluidSystem::ChFsiFluidSystem() : m_is_initialized(false), m_verbose(true), m_step(-1), m_time(0), m_RTF(0) {}

ChFsiFluidSystem::~ChFsiFluidSystem() {}

void ChFsiFluidSystem::SetVerbose(bool verbose) {
    m_verbose = verbose;
}

void ChFsiFluidSystem::SetStepSize(double step) {
    m_step = step;
}

void ChFsiFluidSystem::Initialize(unsigned int num_fsi_bodies,
                                  unsigned int num_fsi_nodes1D,
                                  unsigned int num_fsi_elements1D,
                                  unsigned int num_fsi_nodes2D,
                                  unsigned int num_fsi_elements2D,
                                  const std::vector<FsiBodyState>& body_states,
                                  const std::vector<FsiMeshState>& mesh1D_states,
                                  const std::vector<FsiMeshState>& mesh2D_states,
                                  bool use_node_directions) {
    // Mark system as initialized
    m_is_initialized = true;
}

void ChFsiFluidSystem::Initialize() {
    // Mark system as initialized
    m_is_initialized = true;
}

void ChFsiFluidSystem::DoStepDynamics(double step) {
    m_timer_step.reset();
    m_timer_step.start();

    OnDoStepDynamics(step);

    m_timer_step.stop();
    m_RTF = m_timer_step() / step;
}

}  // end namespace fsi
}  // end namespace chrono
