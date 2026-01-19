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

#include "hydroc/waves/regular_wave.h"
#include "hydroc/waves/irregular_wave.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace fsi {
namespace tdpf {

//------------------------------------------------------------------------------

constexpr int dof_per_body = 6;
constexpr int lin_dof_per_body = 3;

//------------------------------------------------------------------------------

ChFsiFluidSystemTDPF::ChFsiFluidSystemTDPF()
    : ChFsiFluidSystem(),
      m_num_rigid_bodies(0),
      m_num_1D_meshes(0),
      m_num_2D_meshes(0),
      m_num_flex1D_nodes(0),
      m_num_flex2D_nodes(0),
      m_num_flex1D_elements(0),
      m_num_flex2D_elements(0),
      m_convolution_mode(hydrochrono::hydro::RadiationConvolutionMode::Baseline),
      m_waves(std::make_shared<NoWave>()) {}

ChFsiFluidSystemTDPF::~ChFsiFluidSystemTDPF() {}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::SetHydroFilename(const std::string& filename) {
    m_hydro_filename = filename;
}

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
    m_waves = waves;
}

void ChFsiFluidSystemTDPF::AddWaves(const RegularWaveParams& params) {
    m_waves = chrono_types::make_shared<RegularWave>(params);
}

void ChFsiFluidSystemTDPF::AddWaves(const IrregularWaveParams& params) {
    m_waves = chrono_types::make_shared<IrregularWaves>(params);
}

double ChFsiFluidSystemTDPF::GetWaveElevation(const ChVector3d& pos) {
    if (!m_waves)
        return 0;

    return m_waves->GetElevation(pos.eigen(), m_time);
}

void ChFsiFluidSystemTDPF::SetRadiationConvolutionMode(hydrochrono::hydro::RadiationConvolutionMode mode) {
    m_convolution_mode = mode;

    //// RADU - when is it OK to call this so that InvalidateRadiationComponent can be called?
    ////InvalidateRadiationComponent();  // Invalidate component to recreate with new settings
}

void ChFsiFluidSystemTDPF::SetTaperedDirectOptions(const hydrochrono::hydro::TaperedDirectOptions& opts) {
    m_tapered_opts = opts;
    //// RADU - when is it OK to call this so that InvalidateRadiationComponent can be called?
    ////InvalidateRadiationComponent();  // Invalidate component to recreate with new settings
}

//------------------------------------------------------------------------------

void ChFsiFluidSystemTDPF::Initialize(const std::vector<FsiBodyState>& body_states,
                                      const std::vector<FsiMeshState>& mesh1D_states,
                                      const std::vector<FsiMeshState>& mesh2D_states) {
    ChAssertAlways(!m_hydro_filename.empty());

    // Read hydro data from input file
    auto h5_file_info = H5FileInfo(m_hydro_filename, m_num_rigid_bodies);
    try {
        m_hydro_data = H5FileInfo(m_hydro_filename, m_num_rigid_bodies).ReadH5Data();
    } catch (const H5::Exception& e) {
        std::ostringstream oss;
        oss << "Unable to open/read HDF5 hydro data file: " << m_hydro_filename << "\n";
        oss << "HDF5 error: " << e.getDetailMsg() << "\n";
        throw std::runtime_error(oss.str());
    }

    //// RADU - the definitions of various quantities below are private in HydroChrono.
    //// - we include private HydroChrono headers and duplicate some code, or
    //// - we provide an abstraction of a TDPF solver in HydroChrono (separated from HydroSystem)

    // Set up time vector
    m_rirf_time_vector = m_hydro_data.GetRIRFTimeVector();
    // width array
    m_rirf_width_vector.resize(m_rirf_time_vector.size());
    for (Eigen::Index ii = 0; ii < m_rirf_width_vector.size(); ii++) {
        m_rirf_width_vector[ii] = 0.0;
        if (ii < m_rirf_time_vector.size() - 1) {
            m_rirf_width_vector[ii] += 0.5 * abs(m_rirf_time_vector[ii + 1] - m_rirf_time_vector[ii]);
        }
        if (ii > 0) {
            m_rirf_width_vector[ii] += 0.5 * abs(m_rirf_time_vector[ii] - m_rirf_time_vector[ii - 1]);
        }
    }

    // Initialize force vectors
    m_equilibrium.assign(m_num_rigid_bodies * dof_per_body, 0.0);
    m_cb_minus_cg.assign(m_num_rigid_bodies * lin_dof_per_body, 0.0);

    // Compute equilibrium and cb_minus_cg_ (multibody loop)
    for (unsigned int b = 0; b < m_num_rigid_bodies; b++) {
        for (int i = 0; i < lin_dof_per_body; i++) {
            unsigned int eq_idx = i + dof_per_body * b;
            unsigned int c_idx = i + lin_dof_per_body * b;
            m_equilibrium[eq_idx] = m_hydro_data.GetCGVector(b)[i];
            m_cb_minus_cg[c_idx] = m_hydro_data.GetCBVector(b)[i] - m_hydro_data.GetCGVector(b)[i];
        }
    }

    // Initialize waves (NoWave, RegularWave, or IrregularWaves)
    switch (m_waves->GetWaveMode()) { case WaveMode::regular:
            std::static_pointer_cast<RegularWave>(m_waves)->AddH5Data(m_hydro_data.GetRegularWaveInfos(),
                                                                      m_hydro_data.GetSimulationInfo());
            break;
        case WaveMode::irregular:
            std::static_pointer_cast<IrregularWaves>(m_waves)->AddH5Data(m_hydro_data.GetIrregularWaveInfos(),
                                                                         m_hydro_data.GetSimulationInfo());
            break;
    }
    m_waves->Initialize();

    // Build force components for HydroForces
    std::vector<std::unique_ptr<hydrochrono::hydro::IHydroForceComponent>> components;

    // Hydrostatics component (uses shared factory for consistent construction)
    components.push_back(CreateHydrostaticsComponent());

    // Radiation component (uses shared factory for consistent construction)
    components.push_back(CreateRadiationComponent());

    // Excitation component (uses shared factory for consistent construction)
    components.push_back(CreateExcitationComponent());

    // Construct HydroForces (takes ownership of components)
    m_hc_force_system = std::make_unique<hydrochrono::hydro::HydroForces>(m_num_rigid_bodies, std::move(components));

    // Cache initial solid states in the TDPF structure
    m_hc_state.bodies.resize(m_num_rigid_bodies);
    LoadSolidStates(body_states, mesh1D_states, mesh2D_states);
}

std::unique_ptr<hydrochrono::hydro::ExcitationComponent> ChFsiFluidSystemTDPF::CreateExcitationComponent() const {
    return std::make_unique<hydrochrono::hydro::ExcitationComponent>(m_waves, m_num_rigid_bodies);
}

std::unique_ptr<hydrochrono::hydro::HydrostaticsComponent> ChFsiFluidSystemTDPF::CreateHydrostaticsComponent() const {
    return std::make_unique<hydrochrono::hydro::HydrostaticsComponent>(m_hydro_data, m_num_rigid_bodies, m_equilibrium,
                                                                       m_cb_minus_cg, m_g.eigen());
}

std::unique_ptr<hydrochrono::hydro::RadiationComponent> ChFsiFluidSystemTDPF::CreateRadiationComponent() const {
    const int rirf_steps = m_hydro_data.GetRIRFDims(2);
    return std::make_unique<hydrochrono::hydro::RadiationComponent>(
        m_hydro_data, m_num_rigid_bodies, rirf_steps, m_rirf_time_vector, m_rirf_width_vector, m_convolution_mode,
        m_tapered_opts, m_diagnostics_output_dir);
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
