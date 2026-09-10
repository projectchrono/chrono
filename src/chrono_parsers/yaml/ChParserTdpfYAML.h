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
// Authors: Radu Serban
// =============================================================================

#ifndef CH_PARSER_TDPF_YAML_H
#define CH_PARSER_TDPF_YAML_H

#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono_parsers/yaml/ChParserCfdYAML.h"

#include "chrono_fsi/tdpf/ChFsiSystemTDPF.h"
#include "chrono_fsi/tdpf/ChFsiFluidSystemTDPF.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/tdpf/visualization/ChTdpfVisualizationVSG.h"
#endif

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification files for Chrono::TDPF models and simulations.
/// The parser caches model information and simulation settings from a YAML input file and then allows populating an FSI
/// Chrono::TDPF system and setting solver and simulation parameters.
class ChApiParsers ChParserTdpfYAML : public ChParserCfdYAML {
  public:
    ChParserTdpfYAML(const std::string& yamlfilename, bool verbose = false);
    ~ChParserTdpfYAML();

    /// Return true if a YAML solver file has been loaded.
    bool HasSolverData() const { return m_solver_loaded; }

    /// Return true if a YAML model file has been loaded.
    bool HasModelData() const { return m_model_loaded; }

    // --------------

    /// Load the specified MBS simulation input YAML file.
    void LoadFile(const std::string& yaml_filename);

    /// Load the simulation, output, and visualization settings from the specified YAML node.
    void LoadSimData(const YAML::Node& yaml) override;

    /// Load the MBS model from the specified YAML node.
    void LoadModelData(const YAML::Node& yaml);

    /// Load the solver parameters from the specified YAML node.
    /// All sections and keys are optional; any that are absent keep the Chrono::FSI-TDPF defaults.
    /// The expected structure is:
    /// <pre>
    /// radiation:
    ///   method: RIRF_CONVOLUTION       # RIRF_CONVOLUTION (default) or STATE_SPACE
    ///   truncation_time: 0             # [s] truncate the RIRF kernel; 0 = use the full kernel
    ///   smoothing:                     # RIRF_CONVOLUTION only
    ///     type: NONE                   # NONE (default), SG (Savitzky-Golay), or MOVING_AVERAGE
    ///     window_length: 5             # forced to be odd and at least 3
    ///   taper:                         # RIRF_CONVOLUTION only; presence of this block enables tapering
    ///     enabled: true
    ///     start_fraction: 0.8          # taper start, as a fraction of the RIRF length
    ///     end_fraction: 1.0            # taper end, as a fraction of the RIRF length
    ///     final_amplitude: 0.0         # kernel scale at the end of the taper (0 = zero, 1 = unchanged)
    ///   state_space:                   # STATE_SPACE only
    ///     max_order: 10
    ///     r2_threshold: 0.95
    ///     max_hankel_size: 200
    ///     r2_num_samples: 50
    ///   diagnostics:
    ///     export_csv: false            # write before/after RIRF kernels
    /// excitation:
    ///   method: AUTO                   # AUTO (default), IRF_CONVOLUTION, or FREQUENCY_DOMAIN
    ///   interpolation: CARTESIAN       # CARTESIAN (default) or POLAR
    ///   truncation_time: 0             # [s] truncate the excitation IRF; 0 = use the full kernel
    /// diagnostics:
    ///   output_dir: ""                 # directory for solver diagnostics; empty = none
    /// </pre>
    /// Enumeration values are read case-insensitively, so the SEA-Stack spellings (e.g. `state_space`,
    /// `irf_convolution`) are accepted as well.
    void LoadSolverData(const YAML::Node& yaml);

    // --------------

    /// Create and return a Chrono FSI problem configured from cached model and simulation parameters.
    /// By default, the Chrono FSI problem is initialized (with no associated MBS system). If a system is attached after
    /// creation, the caller must create the FSI problem with initialize=false, attach an MBS to the problem, and then
    /// explicitly initialize the FSI problem.
    std::shared_ptr<fsi::tdpf::ChFsiSystemTDPF> CreateFsiSystemTDPF(bool initialize = true);

    /// Access the underlying FSI system.
    virtual std::shared_ptr<fsi::ChFsiSystem> GetFsiSystem() override { return m_sysFSI; }

    /// Access the underlying fluid system.
    virtual std::shared_ptr<fsi::ChFsiFluidSystem> GetFluidSystem() override { return m_sysTDPF; }

    /// Access the underlying fluid system.
    std::shared_ptr<fsi::tdpf::ChFsiFluidSystemTDPF> GetFluidSystemTDPF() { return m_sysTDPF; }

    // --------------

#ifdef CHRONO_VSG
    const fsi::tdpf::ChTdpfVisualizationVSG::Settings& GetTdpfVisualizationSettings() const;
    virtual std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const override;
#endif

    // --------------

    /// Write simulation output results at the current time.
    virtual void WriteOutput(int frame, double time) override;

  private:
    enum class WaveColoringType { NONE, HEIGHT, VELOCITY };

    /// Wave types.
    enum class WaveType { NONE, REGULAR, IRREGULAR };

    /// Output database.
    struct OutputData {
        //// TODO
    };

  private:
    static WaveType ReadWaveType(const YAML::Node& a);
    static fsi::tdpf::ChTdpfRadiationMethod ReadRadiationMethod(const YAML::Node& a);
    static fsi::tdpf::ChTdpfExcitationMethod ReadExcitationMethod(const YAML::Node& a);
    static fsi::tdpf::ChTdpfExcitationInterpolation ReadExcitationInterpolation(const YAML::Node& a);

    /// Report the cached solver settings.
    void PrintSolverInfo() const;

  private:
    fsi::tdpf::ChTdpfSeaState m_sea_state;  ///< sea state settings
    double m_ramp_duration;                 ///< excitation ramp duration [s]; 0 = no ramp

    // Solver settings (from the TDPF solver YAML file)
    fsi::tdpf::ChTdpfRadiationMethod m_radiation_method;              ///< radiation force method
    fsi::tdpf::ChTdpfRadiationKernelProcessing m_kernel_processing;   ///< RIRF kernel smoothing/tapering
    fsi::tdpf::ChTdpfStateSpaceOptions m_state_space_options;         ///< state-space fit settings
    double m_radiation_truncation_time;                               ///< RIRF truncation time [s]; 0 = none
    fsi::tdpf::ChTdpfExcitationMethod m_excitation_method;            ///< wave excitation force method
    fsi::tdpf::ChTdpfExcitationInterpolation m_excitation_interp;     ///< excitation transfer interpolation
    double m_excitation_truncation_time;                              ///< excitation IRF truncation time [s]; 0 = none
    std::string m_diagnostics_output_dir;                             ///< solver diagnostics directory; empty = none

    OutputData m_output_data;  ///< output data
    std::string m_h5_file;     ///< hydrodynamics input file (HDF5 format)
    ChVector3d m_gravity;      ///< gravitational acceleration
    WaveType m_wave_type;      ///< wave type
#ifdef CHRONO_VSG
    fsi::tdpf::ChTdpfVisualizationVSG::Settings m_visTDPF_settings;  ///< TDPF visualization settings
#endif

    std::shared_ptr<fsi::tdpf::ChFsiFluidSystemTDPF> m_sysTDPF;  ///< underlying TDPF fluid solver
    std::shared_ptr<fsi::tdpf::ChFsiSystemTDPF> m_sysFSI;        ///< underlying FSI system

    bool m_loaded;         ///< YAML simulation file loaded
    bool m_solver_loaded;  ///< YAML solver file loaded
    bool m_model_loaded;   ///< YAML model file loaded
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
