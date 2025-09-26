// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#ifndef CH_SPH_PARSER_YAML_H
#define CH_SPH_PARSER_YAML_H

#include "chrono_parsers/yaml/ChParserCfdYAML.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/output/ChOutput.h"
#include "chrono/assets/ChColormap.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/yaml-cpp/include/yaml-cpp/yaml.h"

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Utility class to parse YAML specification files for Chrono::SPH models and simulations.
/// The parser caches model information and simulation settings from the corresponding YAML input files and then allows
/// populating an FSI Chrono::SPH system and setting solver and simulation parameters.
class ChApiParsers ChParserSphYAML : public ChParserCfdYAML {
  public:
    /// Create a YAML parser and load the model from the specified input YAML file.
    ChParserSphYAML(const std::string& yaml_model_filename, const std::string& yaml_sim_filename, bool verbose = false);
    ~ChParserSphYAML();

    /// Return true if a YAML simulation file has been loaded.
    bool HasSimulationData() const { return m_sim_loaded; }

    /// Return true if a YAML model file has been loaded.
    bool HasModelData() const { return m_model_loaded; }

    // --------------

    /// Load the model from the specified input YAML model file.
    void LoadModelFile(const std::string& yaml_filename);

    /// Load the simulation parameters from the specified input YAML simulation file.
    void LoadSimulationFile(const std::string& yaml_filename);

    /// Return the name of the YAML model.
    const std::string& GetName() const { return m_name; }

    double GetTimestep() const { return m_sim.time_step; }
    double GetEndtime() const { return m_sim.end_time; }

    // --------------

    /// Create and return a Chrono FSI problem configured from cached model and simulation parameters.
    /// By default, the Chrono FSI problem is initialized (with no associated MBS system). If a system is attached after
    /// creation, the caller must create the FSI problem with initialize=false, attach an MBS to the problem with
    /// ChFsiProblemSPH::AttachMultibodySystem, and then explictly invoke ChFsiProblemSPH::Initialize().
    std::shared_ptr<fsi::sph::ChFsiProblemSPH> CreateFsiProblemSPH(bool initialize = true);

    // --------------

    bool Render() const { return m_sim.visualization.render; }
    bool UseSplashurf() const { return m_sim.visualization.use_splashsurf; }
#ifdef CHRONO_VSG
    const fsi::sph::ChFsiFluidSystemSPH::SplashsurfParameters& GetSplashsurfParameters() {
        return *m_sim.visualization.splashsurf_params;
    }
    virtual std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const override;
#endif

    // --------------

    ChOutput::Type GetOutputType() const { return m_sim.output.type; }
    double GetOutputFPS() const { return m_sim.output.fps; }

    /// Return true if generating output.
    virtual bool Output() const override { return m_sim.output.type != ChOutput::Type::NONE; }

    /// Set root output directory (default: "").
    void SetOutputDir(const std::string& out_dir) { m_output_dir = out_dir; }

    /// Save simulation output results at the current time.
    virtual void SaveOutput(int frame) override;

  private:  // ---- Data structures
    enum class GeometryType { CARTESIAN, CYLINDRICAL };
    enum class DataPathType { ABS, REL };
    enum class ParticleColoringType { NONE, HEIGHT, VELOCITY, DENSITY, PRESSURE };

    /// Box domain (fluid or container, CARTESIAN).
    struct BoxDomain {
        ChVector3d dimensions;
        ChVector3d origin;
        int wall_code;
    };

    /// Annulus domain (fluid or container, CYLINDRICAL).
    struct AnnulusDomain {
        double inner_radius;
        double outer_radius;
        double height;
        ChVector3d origin;
        int wall_code;
    };

    /// Computational domain.
    struct ComputationalDomain {
        ChAABB aabb;
        fsi::sph::BoundaryConditions bc_type;
    };

    /// Fluid parameters.
    struct FluidParams {
        FluidParams();
        void PrintInfo();

        fsi::sph::PhysicsProblem physics_problem;

        fsi::sph::ChFsiFluidSystemSPH::FluidProperties fluid_props;
        fsi::sph::ChFsiFluidSystemSPH::ElasticMaterialProperties soil_props;

        std::unique_ptr<BoxDomain> fluid_domain_cartesian;
        std::unique_ptr<AnnulusDomain> fluid_domain_cylindrical;
        std::unique_ptr<BoxDomain> container_cartesian;
        std::unique_ptr<AnnulusDomain> container_cylindrical;

        std::unique_ptr<ComputationalDomain> computational_domain;
    };

    /// Run-time visualization parameters.
    struct VisParams {
        VisParams();
        void PrintInfo();

        bool render;
        bool use_splashsurf;

        bool sph_markers;        ///< render fluid SPH particles?
        bool bndry_bce_markers;  ///< render boundary BCE markers?
        bool rigid_bce_markers;  ///< render rigid-body BCE markers?
        bool flex_bce_markers;   ///< render flex-body markers?
        bool active_boxes;       ///< render active boxes?

        ChColormap::Type colormap;  ///< colormap for coloring callback

#ifdef CHRONO_VSG
        std::shared_ptr<fsi::sph::ChSphVisualizationVSG::ParticleColorCallback> color_callback;
        std::shared_ptr<fsi::sph::ChSphVisualizationVSG::MarkerVisibilityCallback> visibility_callback_sph;
        std::shared_ptr<fsi::sph::ChSphVisualizationVSG::MarkerVisibilityCallback> visibility_callback_bce;
        std::unique_ptr<fsi::sph::ChFsiFluidSystemSPH::SplashsurfParameters> splashsurf_params;
#endif

        bool write_images;      ///< if true, save snapshots
        std::string image_dir;  ///< directory for image files
    };

    /// Output parameters.
    struct OutputParameters {
        OutputParameters();
        void PrintInfo();

        ChOutput::Type type;
        ChOutput::Mode mode;
        double fps;
        std::string dir;
    };

    /// Simulation and run-time visualization parameters.
    struct SimParams {
        SimParams();
        void PrintInfo();

        double time_step;
        double end_time;

        ChVector3d gravity;

        fsi::sph::ChFsiFluidSystemSPH::SPHParameters sph;
        VisParams visualization;
        OutputParameters output;
    };

    /// Output database.
    struct OutputData {
        //// TODO
    };

  private:  // ---- Functions
    /// Return the path to the specified data file.
    std::string GetDatafilePath(const std::string& filename);

    static fsi::sph::PhysicsProblem ReadPhysicsProblemType(const YAML::Node& a);
    static GeometryType ReadGeometryType(const YAML::Node& a);
    static DataPathType ReadDataPathType(const YAML::Node& a);
    static fsi::sph::EosType ReadEosType(const YAML::Node& a);
    static fsi::sph::KernelType ReadKernelType(const YAML::Node& a);
    static fsi::sph::IntegrationScheme ReadIntegrationScheme(const YAML::Node& a);
    static fsi::sph::BoundaryMethod ReadBoundaryMethod(const YAML::Node& a);
    static fsi::sph::ShiftingMethod ReadShiftingMethod(const YAML::Node& a);
    static fsi::sph::ViscosityMethod ReadViscosityMethod(const YAML::Node& a);
    static ChOutput::Type ReadOutputType(const YAML::Node& a);
    static ChOutput::Mode ReadOutputMode(const YAML::Node& a);

    static int ReadWallFlagsCartesian(const YAML::Node& a);
    static int ReadWallFlagsCylindrical(const YAML::Node& a);

    static fsi::sph::BCType ReadBoundaryConditionType(const YAML::Node& a);

    static ParticleColoringType ReadParticleColoringType(const YAML::Node& a);
    static ChColormap::Type ReadColorMapType(const YAML::Node& a);
    static fsi::sph::MarkerPlanesVisibilityCallback::Mode ReadVisibilityMode(const YAML::Node& a);

  private:  // ---- Member variables
    GeometryType m_geometry_type;

    FluidParams m_fluid;  ///< fluid parameters
    SimParams m_sim;      ///< simulation parameters

    std::shared_ptr<fsi::sph::ChFsiProblemSPH> m_fsi_problem;  ///< underlying FSI problem

    bool m_depth_based_pressure;
    double m_zero_height;
    bool m_initial_velocity;
    ChVector3d m_velocity;

    std::string m_output_dir;               ///< root oputput directory
    std::shared_ptr<ChOutput> m_output_db;  ///< output database
    OutputData m_output_data;               ///< output data

    bool m_sim_loaded;    ///< YAML simulation file loaded
    bool m_model_loaded;  ///< YAML model file loaded
    std::string m_name;   ///< name of the YAML model

    DataPathType m_data_path;
    std::string m_rel_path;
    std::string m_script_directory;
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
