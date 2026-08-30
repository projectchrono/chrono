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

void ChFsiFluidSystemTDPF::OnAddRigidBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) {
    m_num_rigid_bodies++;
}

#ifdef CHRONO_FEA
void ChFsiFluidSystemTDPF::OnAddFeaMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) {
    m_num_flex1D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex1D_elements += fsi_mesh->GetNumElements();
    m_num_1D_meshes++;
}

void ChFsiFluidSystemTDPF::OnAddFeaMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) {
    m_num_flex2D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex2D_elements += fsi_mesh->GetNumElements();
    m_num_2D_meshes++;
}
#endif

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::SetSeaState(const ChTdpfSeaState& sea_state) {
    m_impl->m_sea_state = sea_state;
}

double ChFsiFluidSystemTDPF::GetWaveElevation(const ChVector3d& pos) {
    return m_impl->GetWaves()->GetElevation(pos.eigen(), m_time);
}

ChVector3d ChFsiFluidSystemTDPF::GetWaveVelocity(const ChVector3d& pos) {
    return m_impl->GetWaves()->GetVelocity(pos.eigen(), m_time);
}

ChVector3d ChFsiFluidSystemTDPF::GetWaveVelocity(const ChVector3d& pos, double elevation) {
    return m_impl->GetWaves()->GetVelocity(pos.eigen(), m_time, elevation);
}

void ChFsiFluidSystemTDPF::SetRadiationMethod(ChTdpfRadiationMethod method) {
    m_impl->m_radiation_method = method;
}

void ChFsiFluidSystemTDPF::SetRadiationKernelProcessing(const ChTdpfRadiationKernelProcessing& opts) {
    m_impl->m_kernel_processing = opts;
}

void ChFsiFluidSystemTDPF::SetStateSpaceOptions(const ChTdpfStateSpaceOptions& opts) {
    m_impl->m_state_space_options = opts;
}

void ChFsiFluidSystemTDPF::SetExcitationMethod(ChTdpfExcitationMethod method) {
    m_impl->m_excitation_method = method;
}

void ChFsiFluidSystemTDPF::SetExcitationInterpolation(ChTdpfExcitationInterpolation interp) {
    m_impl->m_excitation_interpolation = interp;
}

void ChFsiFluidSystemTDPF::SetRampDuration(double seconds) {
    m_impl->m_ramp_duration = seconds;
}

void ChFsiFluidSystemTDPF::SetRadiationTruncationTime(double seconds) {
    m_impl->m_radiation_truncation_time = seconds;
}

void ChFsiFluidSystemTDPF::SetExcitationTruncationTime(double seconds) {
    m_impl->m_excitation_truncation_time = seconds;
}

void ChFsiFluidSystemTDPF::SetDiagnosticsOutputDir(const std::string& dir) {
    m_impl->m_diagnostics_output_dir = dir;
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::Initialize(const std::vector<FsiBodyState>& body_states) {
    ChAssertAlways(!m_hydro_filename.empty());

    // Read hydro data and assemble the hydrodynamic model (waves and force
    // components are created by the SEA-Stack model builder).
    m_impl->Initialize(m_hydro_filename, m_num_rigid_bodies);

    // Cache initial solid states in the TDPF structure
    LoadSolidStates(body_states);
}

#ifdef CHRONO_FEA
void ChFsiFluidSystemTDPF::Initialize(const std::vector<FsiBodyState>& body_states,
                                      const std::vector<FsiMeshState>& mesh1D_states,
                                      const std::vector<FsiMeshState>& mesh2D_states) {
    ChAssertAlways(!m_hydro_filename.empty());

    // Read hydro data and assemble the hydrodynamic model (waves and force
    // components are created by the SEA-Stack model builder).
    m_impl->Initialize(m_hydro_filename, m_num_rigid_bodies);

    // Cache initial solid states in the TDPF structure
    LoadSolidStates(body_states, mesh1D_states, mesh2D_states);
}
#endif

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::LoadSolidStates(const std::vector<FsiBodyState>& body_states) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        m_impl->m_ss_state.bodies[i].position = body_states[i].pos.eigen();
        m_impl->m_ss_state.bodies[i].orientation_rpy = body_states[i].rot.GetCardanAnglesXYZ().eigen();
        m_impl->m_ss_state.bodies[i].linear_velocity = body_states[i].lin_vel.eigen();
        m_impl->m_ss_state.bodies[i].angular_velocity = body_states[i].ang_vel.eigen();
    }
}

void ChFsiFluidSystemTDPF::StoreSolidForces(std::vector<FsiBodyForce>& body_forces) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        body_forces[i].force = m_impl->m_ss_forces[i].force;
        body_forces[i].torque = m_impl->m_ss_forces[i].moment;
    }
}

#ifdef CHRONO_FEA
void ChFsiFluidSystemTDPF::LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                           const std::vector<FsiMeshState>& mesh1D_states,
                                           const std::vector<FsiMeshState>& mesh2D_states) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        m_impl->m_ss_state.bodies[i].position = body_states[i].pos.eigen();
        m_impl->m_ss_state.bodies[i].orientation_rpy = body_states[i].rot.GetCardanAnglesXYZ().eigen();
        m_impl->m_ss_state.bodies[i].linear_velocity = body_states[i].lin_vel.eigen();
        m_impl->m_ss_state.bodies[i].angular_velocity = body_states[i].ang_vel.eigen();
    }
}

void ChFsiFluidSystemTDPF::StoreSolidForces(std::vector<FsiBodyForce>& body_forces, std::vector<FsiMeshForce>& mesh1D_forces, std::vector<FsiMeshForce>& mesh2D_forces) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        body_forces[i].force = m_impl->m_ss_forces[i].force;
        body_forces[i].torque = m_impl->m_ss_forces[i].moment;
    }
}
#endif

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::OnDoStepDynamics(double time, double step) {
    m_impl->CalculateHydroForces(time);
}

void ChFsiFluidSystemTDPF::OnExchangeSolidForces() {}

void ChFsiFluidSystemTDPF::OnExchangeSolidStates() {}

}  // end namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
