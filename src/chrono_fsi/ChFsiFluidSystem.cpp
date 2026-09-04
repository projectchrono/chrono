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
#include <stdexcept>
#include <filesystem>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fsi/ChFsiFluidSystem.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

ChFsiFluidSystem::ChFsiFluidSystem() : m_is_initialized(false), m_verbose(false), m_step(-1), m_time(0), m_frame(0), m_RTF(0), m_node_directions_mode(NodeDirectionsMode::NONE) {}

ChFsiFluidSystem::~ChFsiFluidSystem() {}

void ChFsiFluidSystem::SetVerbose(bool verbose) {
    m_verbose = verbose;
}

void ChFsiFluidSystem::SetStepSize(double step) {
    m_step = step;
}

void ChFsiFluidSystem::Initialize() {
#ifdef CHRONO_FEA
    Initialize(std::vector<FsiBodyState>(), std::vector<FsiMeshState>(), std::vector<FsiMeshState>());
#else
    Initialize(std::vector<FsiBodyState>());
#endif
    // A standalone fluid system (no ChFsiSystem wrapper) is initialized through this call. Mark it
    // here, after the derived Initialize returned without throwing, so that every derived class gets
    // the flag on this path whether or not its own Initialize sets it (ChFsiSystem sets it on the
    // wrapper path). A derived Initialize that throws leaves the flag false and DoStepDynamics refuses.
    m_is_initialized = true;
}

void ChFsiFluidSystem::DoStepDynamics(double step) {
    // Refuse to step an uninitialized system. The wrapper ChFsiSystem::DoStepDynamics already
    // does this; without the same guard here, a standalone fluid system can be stepped before
    // Initialize(), which bypasses CheckSPHParameters() and then dereferences the null
    // m_fluid_dynamics that Initialize() would have constructed. Same message and exception as
    // the wrapper, so a caller handles both identically.
    if (!m_is_initialized) {
        cout << "ERROR: FSI system not initialized!\n" << endl;
        throw std::runtime_error("FSI system not initialized!\n");
    }

    m_timer_step.reset();
    m_timer_step.start();

    OnDoStepDynamics(m_time, step);

    m_timer_step.stop();
    m_RTF = m_timer_step() / step;

    m_frame++;
    m_time += step;
}

}  // end namespace fsi
}  // end namespace chrono
