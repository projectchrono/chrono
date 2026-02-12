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

#include <H5Cpp.h>

#include "chrono_fsi/tdpf/impl/ChFsiFluidSystemTDPF_impl.h"

namespace chrono {
namespace fsi {
namespace tdpf {

//------------------------------------------------------------------------------

constexpr int dof_per_body = 6;
constexpr int lin_dof_per_body = 3;

//------------------------------------------------------------------------------

ChFsiFluidSystemTDPF_impl::ChFsiFluidSystemTDPF_impl()
    : m_num_rigid_bodies(0),
      m_g(0, 0, -9.8),
      m_convolution_mode(hydrochrono::hydro::RadiationConvolutionMode::Baseline),
      m_waves(std::make_shared<NoWave>()) {}

void ChFsiFluidSystemTDPF_impl::Initialize(const std::string& hydro_filename, unsigned int num_bodies) {
    m_num_rigid_bodies = num_bodies;

    // Read hydro data from input file
    auto h5_file_info = H5FileInfo(hydro_filename, m_num_rigid_bodies);
    try {
        m_hydro_data = H5FileInfo(hydro_filename, m_num_rigid_bodies).ReadH5Data();
    } catch (const H5::Exception& e) {
        std::ostringstream oss;
        oss << "Unable to open/read HDF5 hydro data file: " << hydro_filename << "\n";
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
    switch (m_waves->GetWaveMode()) {
        case WaveMode::regular:
            std::static_pointer_cast<RegularWave>(m_waves)->AddH5Data(m_hydro_data.GetRegularWaveInfos(),
                                                                      m_hydro_data.GetSimulationInfo());
            break;
        case WaveMode::irregular:
            std::static_pointer_cast<IrregularWaves>(m_waves)->AddH5Data(m_hydro_data.GetIrregularWaveInfos(),
                                                                         m_hydro_data.GetSimulationInfo());
            break;
    }
    m_waves->Initialize();
}

std::unique_ptr<hydrochrono::hydro::ExcitationComponent> ChFsiFluidSystemTDPF_impl::CreateExcitationComponent() const {
    return std::make_unique<hydrochrono::hydro::ExcitationComponent>(m_waves, m_num_rigid_bodies);
}

std::unique_ptr<hydrochrono::hydro::HydrostaticsComponent> ChFsiFluidSystemTDPF_impl::CreateHydrostaticsComponent()
    const {
    return std::make_unique<hydrochrono::hydro::HydrostaticsComponent>(m_hydro_data, m_num_rigid_bodies, m_equilibrium,
                                                                       m_cb_minus_cg, m_g);
}

std::unique_ptr<hydrochrono::hydro::RadiationComponent> ChFsiFluidSystemTDPF_impl::CreateRadiationComponent() const {
    const int rirf_steps = m_hydro_data.GetRIRFDims(2);
    return std::make_unique<hydrochrono::hydro::RadiationComponent>(
        m_hydro_data, m_num_rigid_bodies, rirf_steps, m_rirf_time_vector, m_rirf_width_vector, m_convolution_mode,
        m_tapered_opts, m_diagnostics_output_dir);
}

void ChFsiFluidSystemTDPF_impl::CalculateHydroForces(double time) {
    m_hc_forces = m_hc_force_system->Evaluate(m_hc_state, time);
}

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
