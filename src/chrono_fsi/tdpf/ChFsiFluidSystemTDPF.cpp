// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
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
// Implementation of an FSI-aware TDPF fluid solver.
//
// =============================================================================

//// #define DEBUG_LOG

#include <cmath>
#include <algorithm>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {
namespace tdpf {

ChFsiFluidSystemTDPF::ChFsiFluidSystemTDPF()
    : ChFsiFluidSystem(),
      m_num_rigid_bodies(0),
      m_num_flex1D_nodes(0),
      m_num_flex2D_nodes(0),
      m_num_flex1D_elements(0),
      m_num_flex2D_elements(0) {}

ChFsiFluidSystemTDPF::~ChFsiFluidSystemTDPF() {}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::SetGravitationalAcceleration(const ChVector3d& gravity) {
    m_g = gravity;
}

ChVector3d ChFsiFluidSystemTDPF::GetGravitationalAcceleration() const {
    return m_g;
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::OnAddFsiBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) {}

void ChFsiFluidSystemTDPF::OnAddFsiMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) {}

void ChFsiFluidSystemTDPF::OnAddFsiMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) {}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::Initialize(const std::vector<FsiBodyState>& body_states,
                                      const std::vector<FsiMeshState>& mesh1D_states,
                                      const std::vector<FsiMeshState>& mesh2D_states) {}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                           const std::vector<FsiMeshState>& mesh1D_states,
                                           const std::vector<FsiMeshState>& mesh2D_states) {}

void ChFsiFluidSystemTDPF::StoreSolidForces(std::vector<FsiBodyForce> body_forces,
                                            std::vector<FsiMeshForce> mesh1D_forces,
                                            std::vector<FsiMeshForce> mesh2D_forces) {}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::OnDoStepDynamics(double time, double step) {}

void ChFsiFluidSystemTDPF::OnExchangeSolidForces() {}

void ChFsiFluidSystemTDPF::OnExchangeSolidStates() {}

}  // end namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
