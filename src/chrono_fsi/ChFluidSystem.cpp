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

#include "chrono_fsi/ChFluidSystem.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

ChFluidSystem::ChFluidSystem()
    : m_is_initialized(false), m_verbose(true), m_step(-1), m_time(0), m_write_mode(OutputMode::NONE) {}

ChFluidSystem::~ChFluidSystem() {}

void ChFluidSystem::SetVerbose(bool verbose) {
    m_verbose = verbose;
}

void ChFluidSystem::SetStepSize(double step) {
    m_step = step;
}

void ChFluidSystem::Initialize(unsigned int num_fsi_bodies,
                               unsigned int num_fsi_nodes1D,
                               unsigned int num_fsi_elements1D,
                               unsigned int num_fsi_nodes2D,
                               unsigned int num_fsi_elements2D) {
    // Mark system as initialized
    m_is_initialized = true;
}

}  // end namespace fsi
}  // end namespace chrono
