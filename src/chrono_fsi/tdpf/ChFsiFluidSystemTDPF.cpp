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

void ChFsiFluidSystemTDPF::OnAddFsiBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) {
    m_num_rigid_bodies++;
}

void ChFsiFluidSystemTDPF::OnAddFsiMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) {
    m_num_flex1D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex1D_elements += fsi_mesh->GetNumElements();
}

void ChFsiFluidSystemTDPF::OnAddFsiMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) {
    m_num_flex2D_nodes += fsi_mesh->GetNumNodes();
    m_num_flex2D_elements += fsi_mesh->GetNumElements();
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::Initialize(const std::vector<FsiBodyState>& body_states,
                                      const std::vector<FsiMeshState>& mesh1D_states,
                                      const std::vector<FsiMeshState>& mesh2D_states) {
    //// RADU - the definitions of various components below are private in HydroChrono.
    //// - we include private HydroChrono headers and duplicate some code, or
    //// - we provide an abstraction of a TDPF solver in HydroChrono (separated from HydroSystem)

    // Build force components for HydroForces
    std::vector<std::unique_ptr<hydrochrono::hydro::IHydroForceComponent>> components;

    /*
    // Hydrostatics component (uses shared factory for consistent construction)
    components.push_back(CreateHydrostaticsComponent());

    // Radiation component (uses shared factory for consistent construction)
    components.push_back(CreateRadiationComponent());

    // Excitation component (uses shared factory for consistent construction)
    components.push_back(CreateExcitationComponent());
    */

    // Construct HydroForces (takes ownership of components)
    m_hc_force_system = std::make_unique<hydrochrono::hydro::HydroForces>(m_num_rigid_bodies, std::move(components));

    // Cache initial solid states in the TDPF structure
    m_hc_state.bodies.resize(m_num_rigid_bodies);
    LoadSolidStates(body_states, mesh1D_states, mesh2D_states);
}

std::unique_ptr<hydrochrono::hydro::ExcitationComponent> ChFsiFluidSystemTDPF::CreateExcitationComponent() const {
    /*
    // Single source of truth for ExcitationComponent construction.
    // Both EnsureExcitationComponent() and EnsureHydroForcesAndCoupler() use this.
    return std::make_unique<hydrochrono::hydro::ExcitationComponent>(user_waves_, num_bodies_);
    */
    return nullptr;
}

std::unique_ptr<hydrochrono::hydro::HydrostaticsComponent> ChFsiFluidSystemTDPF::CreateHydrostaticsComponent() const {
    /*
    // Single source of truth for HydrostaticsComponent construction.
    // Used by HydroSystem constructor and EnsureHydroForcesAndCoupler().
    const auto gravitational_acceleration_ch = bodies_[0]->GetSystem()->GetGravitationalAcceleration();
    const Eigen::Vector3d gravitational_acceleration(
        gravitational_acceleration_ch.x(), gravitational_acceleration_ch.y(), gravitational_acceleration_ch.z());
    return std::make_unique<hydrochrono::hydro::HydrostaticsComponent>(file_info_, num_bodies_, equilibrium_,
                                                                       cb_minus_cg_, gravitational_acceleration);
    */
    return nullptr;
}

std::unique_ptr<hydrochrono::hydro::RadiationComponent> ChFsiFluidSystemTDPF::CreateRadiationComponent() const {
    /*
    // Single source of truth for RadiationComponent construction.
    // Used by EnsureRadiationComponent() and EnsureHydroForcesAndCoupler().
    // Each RadiationComponent instance owns its own velocity history.
    //
    // Configuration inputs:
    //   - file_info_: BEM data including RIRF kernels
    //   - num_bodies_: number of bodies in system
    //   - rirf_time_vector, rirf_width_vector: time discretization for RIRF
    //   - convolution_mode_: Baseline or TaperedDirect
    //   - tapered_opts_: preprocessing options for TaperedDirect mode
    //   - diagnostics_output_dir_: where to write debug CSVs

    const int rirf_steps = file_info_.GetRIRFDims(2);

    // convolution_mode_ and tapered_opts_ are now the canonical types from
    // hydrochrono::hydro namespace - no conversion needed (single source of truth)
    return std::make_unique<hydrochrono::hydro::RadiationComponent>(
        file_info_, num_bodies_, rirf_steps, rirf_time_vector, rirf_width_vector, convolution_mode_, tapered_opts_,
        diagnostics_output_dir_);
    */
    return nullptr;
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                           const std::vector<FsiMeshState>& mesh1D_states,
                                           const std::vector<FsiMeshState>& mesh2D_states) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        m_hc_state.bodies[i].position = body_states[i].pos.eigen();
        m_hc_state.bodies[i].orientation_rpy = body_states[i].rot.GetCardanAnglesXYZ().eigen();
        m_hc_state.bodies[i].linear_velocity = body_states[i].lin_vel.eigen();
        m_hc_state.bodies[i].angular_velocity = body_states[i].ang_vel.eigen();
    }
}

void ChFsiFluidSystemTDPF::StoreSolidForces(std::vector<FsiBodyForce> body_forces,
                                            std::vector<FsiMeshForce> mesh1D_forces,
                                            std::vector<FsiMeshForce> mesh2D_forces) {
    for (unsigned int i = 0; i < m_num_rigid_bodies; i++) {
        body_forces[i].force = m_hc_forces[i].segment(0, 3);
        body_forces[i].torque = m_hc_forces[i].segment(3, 3);
    }
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::OnDoStepDynamics(double time, double step) {
    // Calculate TDPF solid forces
    m_hc_forces = m_hc_force_system->Evaluate(m_hc_state, time);
}

void ChFsiFluidSystemTDPF::OnExchangeSolidForces() {}

void ChFsiFluidSystemTDPF::OnExchangeSolidStates() {}

}  // end namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
