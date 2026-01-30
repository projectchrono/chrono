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

#include "chrono/assets/ChColormap.h"

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
/// The parser caches model information and simulation settings from the corresponding YAML input files and then allows
/// populating an FSI Chrono::TDPF system and setting solver and simulation parameters.
class ChApiParsers ChParserTdpfYAML : public ChParserCfdYAML {
  public:
    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserTdpfYAML(const std::string& yaml_model_filename,
                     const std::string& yaml_sim_filename,
                     bool verbose = false);
    ~ChParserTdpfYAML();

    /// Return true if a YAML simulation file has been loaded.
    bool HasSimulationData() const { return m_sim_loaded; }

    /// Return true if a YAML model file has been loaded.
    bool HasModelData() const { return m_model_loaded; }

    // --------------

    /// Load the model from the specified input YAML model file.
    void LoadModelFile(const std::string& yaml_filename);

    /// Load the simulation parameters from the specified input YAML simulation file.
    void LoadSimulationFile(const std::string& yaml_filename);

    // --------------

    /// Create and return a Chrono FSI problem configured from cached model and simulation parameters.
    /// By default, the Chrono FSI problem is initialized (with no associated MBS system). If a system is attached after
    /// creation, the caller must create the FSI problem with initialize=false, attach an MBS to the problem, and then
    /// explictly initialize the FSI problem.
    std::shared_ptr<fsi::tdpf::ChFsiSystemTDPF> CreateFsiSystemTDPF(bool initialize = true);

    /// Access the underlying FSI system.
    virtual std::shared_ptr<fsi::ChFsiSystem> GetFsiSystem() override { return m_sysFSI; }

    /// Access the underlying fluid system.
    virtual std::shared_ptr<fsi::ChFsiFluidSystem> GetFluidSystem() override { return m_sysTDPF; }

    // --------------

    bool Render() const { return m_vis.render; }
#ifdef CHRONO_VSG
    virtual std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const override;
#endif

    // --------------

    /// Save simulation output results at the current time.
    virtual void SaveOutput(int frame) override;

  private:  // ---- Data structures
    enum class WaveColoringType { NONE, HEIGHT, VELOCITY };

    /// Wave types.
    enum class WaveType { NONE, REGULAR, IRREGULAR };

    /// Run-time visualization parameters.
    struct VisParams {
        VisParams();
        void PrintInfo();

        bool render;

        fsi::tdpf::ChTdpfVisualizationVSG::ColorMode mode;  ///< mode for wave false coloring
        ChColormap::Type colormap;                          ///< colormap for wave false coloring
        ChVector2d range;                                   ///< data range for false coloring
        double update_fps;                                  ///< wave mesh update frequency (in FPS)

        bool write_images;      ///< if true, save snapshots
        std::string image_dir;  ///< directory for image files
    };

    /// Output database.
    struct OutputData {
        //// TODO
    };

  private:
    static ChColormap::Type ReadColorMapType(const YAML::Node& a);
    static WaveType ReadWaveType(const YAML::Node& a);
    static fsi::tdpf::ChTdpfVisualizationVSG::ColorMode ReadWaveColoringMode(const YAML::Node& a);

    RegularWaveParams m_reg_wave_params;
    IrregularWaveParams m_irreg_wave_params;

    VisParams m_vis;                                             ///< visualization parameters
    OutputData m_output_data;                                    ///< output data
    std::string m_h5_file;                                       ///< hydrodynamics input file (HDF5 format)
    ChVector3d m_gravity;                                        ///< gravitational acceleration
    WaveType m_wave_type;                                        ///< wave type
    std::shared_ptr<fsi::tdpf::ChFsiFluidSystemTDPF> m_sysTDPF;  ///< underlying TDPF fluid solver
    std::shared_ptr<fsi::tdpf::ChFsiSystemTDPF> m_sysFSI;        ///< underlying FSI system

    bool m_sim_loaded;    ///< YAML simulation file loaded
    bool m_model_loaded;  ///< YAML model file loaded
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
