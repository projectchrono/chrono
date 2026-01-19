// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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

#include <cmath>
#include <algorithm>

#include <H5Cpp.h>

#include "chrono/core/ChTypes.h"

#include "chrono/utils/ChUtils.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

#include "chrono_fsi/tdpf/impl/ChFsiFluidSystemTDPF_impl.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {
namespace tdpf {

ChFsiFluidSystemTDPF::ChFsiFluidSystemTDPF()
    : ChFsiFluidSystem(),
      m_num_rigid_bodies(0),
      m_num_1D_meshes(0),
      m_num_2D_meshes(0),
      m_num_flex1D_nodes(0),
      m_num_flex2D_nodes(0),
      m_num_flex1D_elements(0),
      m_num_flex2D_elements(0),
      m_impl(chrono_types::make_unique<ChFsiFluidSystemTDPF_impl>()) {}

ChFsiFluidSystemTDPF::~ChFsiFluidSystemTDPF() {}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::SetHydroFilename(const std::string& filename) {
    m_hydro_filename = filename;
}

void ChFsiFluidSystemTDPF::SetGravitationalAcceleration(const ChVector3d& gravity) {
    m_impl->m_g = gravity.eigen();
}

ChVector3d ChFsiFluidSystemTDPF::GetGravitationalAcceleration() const {
    return ChVector3d(m_impl->m_g);
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::OnAddFsiBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) {
    m_num_rigid_bodies++;
}

void ChFsiFluidSystemTDPF::OnAddFsiMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) {
    m_num_flex1D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex1D_elements += fsi_mesh->GetNumElements();
    m_num_1D_meshes++;
}

void ChFsiFluidSystemTDPF::OnAddFsiMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) {
    m_num_flex2D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex2D_elements += fsi_mesh->GetNumElements();
    m_num_2D_meshes++;
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::AddWaves(const NoWaveParams& params) {
    auto waves = chrono_types::make_shared<NoWave>(params);
    m_impl->m_waves = waves;
}

void ChFsiFluidSystemTDPF::AddWaves(const RegularWaveParams& params) {
    m_impl->m_waves = chrono_types::make_shared<RegularWave>(params);
}

void ChFsiFluidSystemTDPF::AddWaves(const IrregularWaveParams& params) {
    m_impl->m_waves = chrono_types::make_shared<IrregularWaves>(params);
}

double ChFsiFluidSystemTDPF::GetWaveElevation(const ChVector3d& pos) {
    return m_impl->m_waves->GetElevation(pos.eigen(), m_time);
}

ChVector3d ChFsiFluidSystemTDPF::GetWaveVelocity(const ChVector3d& pos) {
    return m_impl->m_waves->GetVelocity(pos.eigen(), m_time);
}

ChVector3d ChFsiFluidSystemTDPF::GetWaveVelocity(const ChVector3d& pos, double elevation) {
    return m_impl->m_waves->GetVelocity(pos.eigen(), m_time, elevation);
}

void ChFsiFluidSystemTDPF::SetRadiationConvolutionMode(hydrochrono::hydro::RadiationConvolutionMode mode) {
    m_impl->m_convolution_mode = mode;

    //// RADU - when is it OK to call this so that InvalidateRadiationComponent can be called?
    ////InvalidateRadiationComponent();  // Invalidate component to recreate with new settings
}

void ChFsiFluidSystemTDPF::SetTaperedDirectOptions(const hydrochrono::hydro::TaperedDirectOptions& opts) {
    m_impl->m_tapered_opts = opts;
    //// RADU - when is it OK to call this so that InvalidateRadiationComponent can be called?
    ////InvalidateRadiationComponent();  // Invalidate component to recreate with new settings
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::Initialize(const std::vector<FsiBodyState>& body_states,
                                      const std::vector<FsiMeshState>& mesh1D_states,
                                      const std::vector<FsiMeshState>& mesh2D_states) {
    ChAssertAlways(!m_hydro_filename.empty());

    // Initialize low-level implementation object
    m_impl->Initialize(m_hydro_filename, m_num_rigid_bodies);

    // Build force components for HydroForces
    std::vector<std::unique_ptr<hydrochrono::hydro::IHydroForceComponent>> components;

    // Hydrostatics component (uses shared factory for consistent construction)
    components.push_back(m_impl->CreateHydrostaticsComponent());

    // Radiation component (uses shared factory for consistent construction)
    components.push_back(m_impl->CreateRadiationComponent());

    // Excitation component (uses shared factory for consistent construction)
    components.push_back(m_impl->CreateExcitationComponent());

    // Construct HydroForces (takes ownership of components)
    m_impl->m_hc_force_system =
        std::make_unique<hydrochrono::hydro::HydroForces>(m_num_rigid_bodies, std::move(components));

    // Cache initial solid states in the TDPF structure
    m_impl->m_hc_state.bodies.resize(m_num_rigid_bodies);
    LoadSolidStates(body_states, mesh1D_states, mesh2D_states);
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                           const std::vector<FsiMeshState>& mesh1D_states,
                                           const std::vector<FsiMeshState>& mesh2D_states) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        m_impl->m_hc_state.bodies[i].position = body_states[i].pos.eigen();
        m_impl->m_hc_state.bodies[i].orientation_rpy = body_states[i].rot.GetCardanAnglesXYZ().eigen();
        m_impl->m_hc_state.bodies[i].linear_velocity = body_states[i].lin_vel.eigen();
        m_impl->m_hc_state.bodies[i].angular_velocity = body_states[i].ang_vel.eigen();
    }
}

void ChFsiFluidSystemTDPF::StoreSolidForces(std::vector<FsiBodyForce> body_forces,
                                            std::vector<FsiMeshForce> mesh1D_forces,
                                            std::vector<FsiMeshForce> mesh2D_forces) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        body_forces[i].force = m_impl->m_hc_forces[i].segment(0, 3);
        body_forces[i].torque = m_impl->m_hc_forces[i].segment(3, 3);
    }
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::OnDoStepDynamics(double time, double step) {
    m_impl->CalculateHydroForces(time);
}

void ChFsiFluidSystemTDPF::OnExchangeSolidForces() {}

void ChFsiFluidSystemTDPF::OnExchangeSolidStates() {}

}  // end namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
