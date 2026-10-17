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

#include <sstream>
#include <stdexcept>

#include <seastack/hydro_io/h5_reader.h>

#include "chrono_fsi/tdpf/impl/ChFsiFluidSystemTDPF_impl.h"

namespace chrono {
namespace fsi {
namespace tdpf {

namespace ss = seastack::hydro;

//------------------------------------------------------------------------------
// Translation from the Chrono-native configuration types to SEA-Stack types.
//------------------------------------------------------------------------------

namespace {

ss::SeaStateDefinition ToSeaStack(const ChTdpfSeaState& s) {
    ss::SeaStateDefinition def;
    def.type = s.type;
    def.depth = s.depth;
    def.g = s.g;
    def.amplitude = s.amplitude;
    def.omega = s.omega;
    def.direction_deg = s.direction_deg;
    def.phase_rad = s.phase_rad;
    def.eta_file_path = s.eta_file_path;
    def.n_omega = s.n_omega;
    def.n_theta = s.n_theta;
    def.omega_min = s.omega_min;
    def.omega_max = s.omega_max;
    def.seed = s.seed;

    def.partitions.reserve(s.partitions.size());
    for (const auto& p : s.partitions) {
        ss::SeaStatePartition part;
        part.spectrum.type = p.spectrum.type;
        part.spectrum.Hs = p.spectrum.Hs;
        part.spectrum.Tp = p.spectrum.Tp;
        part.spectrum.gamma = p.spectrum.gamma;
        part.spreading.type = p.spreading.type;
        part.spreading.mean_direction_deg = p.spreading.mean_direction_deg;
        part.spreading.s = p.spreading.s;
        def.partitions.push_back(part);
    }

    return def;
}

ss::RadiationKernelProcessing ToSeaStack(const ChTdpfRadiationKernelProcessing& k) {
    ss::RadiationKernelProcessing opts;
    opts.smoothing_type = k.smoothing_type;
    opts.smoothing_window = k.smoothing_window;
    opts.taper_enabled = k.taper_enabled;
    opts.taper_start_fraction = k.taper_start_fraction;
    opts.taper_end_fraction = k.taper_end_fraction;
    opts.taper_final_amplitude = k.taper_final_amplitude;
    opts.export_csv = k.export_csv;
    return opts;
}

ss::StateSpaceOptions ToSeaStack(const ChTdpfStateSpaceOptions& o) {
    ss::StateSpaceOptions opts;
    opts.max_order = o.max_order;
    opts.r2_threshold = o.r2_threshold;
    opts.max_hankel_size = o.max_hankel_size;
    opts.r2_num_samples = o.r2_num_samples;
    return opts;
}

ss::RadiationMethod ToSeaStack(ChTdpfRadiationMethod m) {
    switch (m) {
        case ChTdpfRadiationMethod::STATE_SPACE:
            return ss::RadiationMethod::kStateSpace;
        case ChTdpfRadiationMethod::RIRF_CONVOLUTION:
        default:
            return ss::RadiationMethod::kRirfConvolution;
    }
}

ss::ExcitationMethod ToSeaStack(ChTdpfExcitationMethod m) {
    switch (m) {
        case ChTdpfExcitationMethod::IRF_CONVOLUTION:
            return ss::ExcitationMethod::kIrfConvolution;
        case ChTdpfExcitationMethod::FREQUENCY_DOMAIN:
            return ss::ExcitationMethod::kFrequencyDomain;
        case ChTdpfExcitationMethod::AUTO:
        default:
            return ss::ExcitationMethod::kAuto;
    }
}

ss::ExcitationInterpolation ToSeaStack(ChTdpfExcitationInterpolation i) {
    switch (i) {
        case ChTdpfExcitationInterpolation::POLAR:
            return ss::ExcitationInterpolation::kPolar;
        case ChTdpfExcitationInterpolation::CARTESIAN:
        default:
            return ss::ExcitationInterpolation::kCartesian;
    }
}

/// True if the requested sea state represents still water.
bool IsStillWater(const ChTdpfSeaState& s) {
    return s.type.empty() || s.type == "none" || s.type == "no_wave";
}

}  // namespace

//------------------------------------------------------------------------------

ChFsiFluidSystemTDPF_impl::ChFsiFluidSystemTDPF_impl()
    : m_num_rigid_bodies(0),
      m_g(0, 0, -9.8),
      m_radiation_method(ChTdpfRadiationMethod::RIRF_CONVOLUTION),
      m_excitation_method(ChTdpfExcitationMethod::AUTO),
      m_excitation_interpolation(ChTdpfExcitationInterpolation::CARTESIAN),
      m_ramp_duration(0),
      m_radiation_truncation_time(0),
      m_excitation_truncation_time(0) {}

ChFsiFluidSystemTDPF_impl::~ChFsiFluidSystemTDPF_impl() {}

const seastack::hydro::HydroData& ChFsiFluidSystemTDPF_impl::GetHydroData() const {
    return m_model->GetData();
}

void ChFsiFluidSystemTDPF_impl::Initialize(const std::string& hydro_filename, unsigned int num_bodies) {
    m_num_rigid_bodies = num_bodies;

    // Read hydro coefficient data from the BEMIO-format HDF5 input file.
    ss::HydroData hydro_data;
    try {
        hydro_data = seastack::hydro_io::H5FileInfo(hydro_filename, (int)m_num_rigid_bodies).ReadH5Data();
    } catch (const std::exception& e) {
        std::ostringstream oss;
        oss << "Unable to open/read HDF5 hydro data file: " << hydro_filename << "\n";
        oss << "Error: " << e.what() << "\n";
        throw std::runtime_error(oss.str());
    }

    // Assemble the hydrodynamic model. The builder performs the setup ceremony
    // (equilibrium, cb - cg, RIRF widths, wave initialization, and force
    // component construction).
    ss::HydroModelBuilder builder;
    builder.FromHydroData(std::move(hydro_data));

    if (IsStillWater(m_sea_state)) {
        // Still water: install an explicit no-wave model. The builder then skips
        // creation of the excitation component (its contribution is zero).
        auto no_wave = std::make_shared<ss::NoWave>();
        no_wave->SetNumBodies(m_num_rigid_bodies);
        builder.WithWave(no_wave);
    } else {
        builder.WithSeaState(ToSeaStack(m_sea_state));
    }

    builder.EnableHydrostatics();
    builder.EnableRadiation();
    builder.EnableExcitation();

    builder.WithGravity(m_g);

    builder.WithRadiationMethod(ToSeaStack(m_radiation_method));
    builder.WithRadiationOptions(ToSeaStack(m_kernel_processing));
    builder.WithStateSpaceOptions(ToSeaStack(m_state_space_options));

    builder.WithExcitationMethod(ToSeaStack(m_excitation_method));
    builder.WithExcitationInterpolation(ToSeaStack(m_excitation_interpolation));

    if (m_ramp_duration > 0)
        builder.WithRampDuration(m_ramp_duration);
    if (m_radiation_truncation_time > 0)
        builder.WithRadiationTruncationTime(m_radiation_truncation_time);
    if (m_excitation_truncation_time > 0)
        builder.WithExcitationTruncationTime(m_excitation_truncation_time);
    if (!m_diagnostics_output_dir.empty())
        builder.WithDiagnosticsOutputDir(m_diagnostics_output_dir);

    m_model = std::make_unique<ss::HydroModel>(builder.Build());
    m_waves = m_model->GetWave();

    // Size the exchange buffers.
    m_ss_state.bodies.resize(m_num_rigid_bodies);
    m_ss_forces.assign(m_num_rigid_bodies, ss::GeneralizedForce());
}

void ChFsiFluidSystemTDPF_impl::CalculateHydroForces(double time) {
    m_ss_forces = m_model->Evaluate(m_ss_state, time);
}

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono
