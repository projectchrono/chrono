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
// Private implementation of the TDPF fluid solver, wrapping SEA-Stack.
//
// This is the only place where SEA-Stack headers are included; no public Chrono
// header refers to any seastack:: type.
//
// =============================================================================

#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

#include <seastack/core/system_state.h>
#include <seastack/core/types.h>
#include <seastack/hydro/hydro_model_builder.h>
#include <seastack/hydro/waves/wave_base.h>

#include "chrono_fsi/tdpf/ChFsiTdpfTypes.h"

namespace chrono {
namespace fsi {
namespace tdpf {

class ChFsiFluidSystemTDPF_impl {
  public:
    ChFsiFluidSystemTDPF_impl();
    ~ChFsiFluidSystemTDPF_impl();

  private:
    /// Read the hydro data file and assemble the SEA-Stack hydrodynamic model.
    /// All force components are created here through HydroModelBuilder.
    void Initialize(const std::string& hydro_filename, unsigned int num_bodies);

    /// Evaluate hydrodynamic forces for the cached solid state at the given time.
    void CalculateHydroForces(double time);

    /// Access the hydrodynamic coefficient data owned by the model.
    /// Only valid after Initialize().
    const seastack::hydro::HydroData& GetHydroData() const;

    /// Access the wave model owned by the model (never null after Initialize()).
    const std::shared_ptr<seastack::hydro::WaveBase>& GetWaves() const { return m_waves; }

    unsigned int m_num_rigid_bodies;  ///< number of rigid bodies
    Eigen::Vector3d m_g;              ///< gravitational acceleration

    // Solver configuration, staged before Initialize() and translated to
    // SEA-Stack types there.
    ChTdpfSeaState m_sea_state;
    ChTdpfRadiationMethod m_radiation_method;
    ChTdpfRadiationKernelProcessing m_kernel_processing;
    ChTdpfStateSpaceOptions m_state_space_options;
    ChTdpfExcitationMethod m_excitation_method;
    ChTdpfExcitationInterpolation m_excitation_interpolation;
    double m_ramp_duration;
    double m_radiation_truncation_time;
    double m_excitation_truncation_time;
    std::string m_diagnostics_output_dir;

    // SEA-Stack model. Owns the HydroData, the wave object, and the force
    // components; force components hold a reference to the HydroData, so the
    // model must outlive any force evaluation.
    std::unique_ptr<seastack::hydro::HydroModel> m_model;
    std::shared_ptr<seastack::hydro::WaveBase> m_waves;

    // Exchange buffers.
    seastack::hydro::SystemState m_ss_state;
    seastack::hydro::BodyForces m_ss_forces;

    friend class ChFsiSystemTDPF;
    friend class ChFsiFluidSystemTDPF;
    friend class ChFsiInterfaceTDPF;
};

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
