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
// Chrono-native configuration types for the TDPF fluid solver.
//
// These structures mirror the corresponding declarative types in SEA-Stack
// (seastack::hydro::SeaStateDefinition, RadiationMethod,
// RadiationKernelProcessing, StateSpaceOptions, ExcitationMethod, and
// ExcitationInterpolation). They are duplicated here so that no SEA-Stack
// header is required by any public Chrono header; translation happens in
// impl/ChFsiFluidSystemTDPF_impl.cpp.
//
// Field names, units, and default values intentionally follow SEA-Stack.
// Note that all frequencies are angular frequencies [rad/s].
//
// =============================================================================

#ifndef CH_FSI_TDPF_TYPES_H
#define CH_FSI_TDPF_TYPES_H

#include <string>
#include <vector>

#include "chrono_fsi/ChApiFsi.h"

namespace chrono {
namespace fsi {
namespace tdpf {

/// @addtogroup fsitdpf
/// @{

// -----------------------------------------------------------------------------
// Sea state description
// -----------------------------------------------------------------------------

/// Frequency spectrum definition for one wave train.
struct ChTdpfSpectrum {
    std::string type = "jonswap";  ///< "jonswap" or "pierson_moskowitz"
    double Hs = 0.0;               ///< significant wave height [m]
    double Tp = 0.0;               ///< peak period [s]
    double gamma = 3.3;            ///< JONSWAP peak enhancement factor (ignored for "pierson_moskowitz")
};

/// Directional spreading definition for one wave train.
struct ChTdpfSpreading {
    std::string type = "none";        ///< "none" (long-crested) or "cos2s"
    double mean_direction_deg = 0.0;  ///< mean propagation direction [deg], 0 = +X
    double s = 12.0;                  ///< spreading parameter for "cos2s"
};

/// A single spectral partition (one wave train of a possibly multi-modal sea state).
struct ChTdpfSeaStatePartition {
    ChTdpfSpectrum spectrum;
    ChTdpfSpreading spreading;
};

/// Complete declarative description of a sea state.
/// Mirrors seastack::hydro::SeaStateDefinition.
struct ChTdpfSeaState {
    /// Sea state kind: "none" (still water), "regular", or "irregular".
    /// "none" installs a no-wave model; excitation forces are then identically zero.
    std::string type = "none";

    double depth = 0.0;  ///< water depth [m]; 0 = take value from the HDF5 file
    double g = 0.0;      ///< gravitational acceleration [m/s^2]; 0 = take value from the HDF5 file

    // --- Regular wave parameters (used when type == "regular") ---
    double amplitude = 0.0;      ///< wave amplitude [m]
    double omega = 0.0;          ///< angular frequency [rad/s]
    double direction_deg = 0.0;  ///< propagation direction [deg], 0 = +X
    double phase_rad = 0.0;      ///< phase [rad]

    // --- Spectral partitions (used when type == "irregular") ---
    /// One partition for a unimodal sea state; two or more for bimodal/multi-modal.
    std::vector<ChTdpfSeaStatePartition> partitions;

    // --- Elevation file import (alternative to spectral generation) ---
    /// If set, wave components are extracted from this elevation time series via DFT
    /// instead of being sampled from a parametric spectrum.
    std::string eta_file_path;

    // --- Discretization ---
    int n_omega = 512;       ///< number of frequency bins
    int n_theta = 1;         ///< number of directional bins (1 = long-crested)
    double omega_min = 0.0;  ///< minimum angular frequency [rad/s]; 0 = auto
    double omega_max = 0.0;  ///< maximum angular frequency [rad/s]; 0 = auto

    int seed = 42;  ///< random seed for phase generation
};

// -----------------------------------------------------------------------------
// Radiation options
// -----------------------------------------------------------------------------

/// Radiation damping calculation method.
enum class ChTdpfRadiationMethod {
    RIRF_CONVOLUTION,  ///< direct convolution of RIRF kernels with velocity history (default)
    STATE_SPACE        ///< state-space approximation using exponential decay modes
};

/// RIRF kernel processing (smoothing and/or tapering).
/// Defaults are no-ops. Only used with ChTdpfRadiationMethod::RIRF_CONVOLUTION.
struct ChTdpfRadiationKernelProcessing {
    std::string smoothing_type = "none";  ///< "none", "sg" (Savitzky-Golay), or "moving_average"
    int smoothing_window = 5;             ///< smoothing window length (odd, >= 3)

    bool taper_enabled = false;          ///< apply a half-cosine taper near the end of the RIRF
    double taper_start_fraction = 0.8;   ///< start taper at this fraction [0,1] of the RIRF length
    double taper_end_fraction = 1.0;     ///< end taper at this fraction [0,1] of the RIRF length
    double taper_final_amplitude = 0.0;  ///< final amplitude (0 = zero, 1 = no change)

    bool export_csv = false;  ///< export before/after CSV for diagnostics
};

/// Settings for the state-space radiation approximation.
/// Only used with ChTdpfRadiationMethod::STATE_SPACE.
struct ChTdpfStateSpaceOptions {
    int max_order = 10;           ///< maximum number of state-space modes per DOF pair
    double r2_threshold = 0.95;   ///< minimum R^2 for fit acceptance
    int max_hankel_size = 200;    ///< maximum Hankel matrix size for SVD
    int r2_num_samples = 50;      ///< number of subsamples used for the R^2 quality check
};

// -----------------------------------------------------------------------------
// Excitation options
// -----------------------------------------------------------------------------

/// Wave excitation force calculation method.
enum class ChTdpfExcitationMethod {
    AUTO,             ///< frequency-domain for regular and multi-heading irregular seas; IRF otherwise
    IRF_CONVOLUTION,  ///< time-domain convolution with the excitation IRF
    FREQUENCY_DOMAIN  ///< frequency-domain superposition of the excitation transfer function
};

/// Interpolation method for the excitation transfer function read from the HDF5 file.
enum class ChTdpfExcitationInterpolation {
    CARTESIAN,  ///< Re/Im interpolation (default; avoids phase-wrap artefacts)
    POLAR       ///< magnitude/phase interpolation (matches legacy HydroChrono behavior)
};

/// @} fsitdpf

}  // namespace tdpf
}  // end namespace fsi
}  // end namespace chrono

#endif
