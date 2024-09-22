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
// Base class for an FSI system, independent of the fluid solver
//
// =============================================================================

#include <cmath>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fsi/ChFsiSystem.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

using namespace rapidjson;

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {

ChFsiSystem::ChFsiSystem(ChSystem* sysMBS)
    : m_sysMBS(sysMBS), m_verbose(true), m_RTF(0), m_ratio_MBS(0), m_write_mode(OutputMode::NONE) {}

ChFsiSystem::~ChFsiSystem() {}

void ChFsiSystem::AttachSystem(ChSystem* sysMBS) {
    m_sysMBS = sysMBS;
}

void ChFsiSystem::SetVerbose(bool verbose) {
    m_verbose = verbose;
    m_fsi_interface->m_verbose = verbose;
}

void ChFsiSystem::Initialize() {
    if (!m_fsi_interface) {
        throw std::runtime_error("No FSI interface was created.");
    }
}

const ChVector3d& ChFsiSystem::GetFsiBodyForce(size_t i) const {
    return m_fsi_interface->m_fsi_bodies[i].fsi_force;
}

const ChVector3d& ChFsiSystem::GetFsiBodyTorque(size_t i) const {
    return m_fsi_interface->m_fsi_bodies[i].fsi_torque;
}

}  // end namespace fsi
}  // end namespace chrono
