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

#pragma once

#include "hydroc/core/hydro_types.h"
#include "hydroc/core/system_state.h"
#include "hydroc/core/hydro_forces.h"

#include "hydro/force_components/excitation_component.h"
#include "hydro/force_components/hydrostatics_component.h"
#include "hydro/force_components/radiation_component.h"

#include "hydroc/waves/regular_wave.h"
#include "hydroc/waves/irregular_wave.h"

#include "hydroc/io/h5_reader.h"

namespace chrono {
namespace fsi {
namespace tdpf {

class ChFsiFluidSystemTDPF_impl {
  public:
    ChFsiFluidSystemTDPF_impl();
    ~ChFsiFluidSystemTDPF_impl() {}

  private:
    void Initialize(const std::string& hydro_filename, unsigned int num_bodies);

    std::unique_ptr<hydrochrono::hydro::ExcitationComponent> CreateExcitationComponent() const;
    std::unique_ptr<hydrochrono::hydro::HydrostaticsComponent> CreateHydrostaticsComponent() const;
    std::unique_ptr<hydrochrono::hydro::RadiationComponent> CreateRadiationComponent() const;

    void CalculateHydroForces(double time);

    HydroData m_hydro_data;

    unsigned int m_num_rigid_bodies;
    Eigen::Vector3d m_g;

    // Additional properties related to equilibrium and hydrodynamics
    std::vector<double> m_equilibrium;
    std::vector<double> m_cb_minus_cg;
    Eigen::VectorXd m_rirf_time_vector;
    Eigen::VectorXd m_rirf_width_vector;

    hydrochrono::hydro::RadiationConvolutionMode m_convolution_mode;
    hydrochrono::hydro::TaperedDirectOptions m_tapered_opts;
    std::string m_diagnostics_output_dir;

    std::shared_ptr<WaveBase> m_waves;

    hydrochrono::hydro::SystemState m_hc_state;
    hydrochrono::hydro::BodyForces m_hc_forces;
    std::unique_ptr<hydrochrono::hydro::HydroForces> m_hc_force_system;

    friend class ChFsiSystemTDPF;
    friend class ChFsiFluidSystemTDPF;
    friend class ChFsiInterfaceTDPF;
};

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
