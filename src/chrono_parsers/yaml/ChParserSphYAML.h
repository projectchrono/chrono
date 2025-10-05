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

#ifndef CH_PARSER_SPH_YAML_H
#define CH_PARSER_SPH_YAML_H

#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono_parsers/yaml/ChParserCfdYAML.h"

#include "chrono/assets/ChColormap.h"

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification files for Chrono::SPH models and simulations.
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

    /// Save simulation output results at the current time.
    virtual void SaveOutput(int frame) override;

  private:  // ---- Data structures
    enum class GeometryType { CARTESIAN, CYLINDRICAL };
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

    /// Material (fluid or soil) properties
    struct MaterialProperties {
        MaterialProperties();
        void PrintInfo();

        fsi::sph::PhysicsProblem physics_problem;
        fsi::sph::ChFsiFluidSystemSPH::FluidProperties fluid_props;
        fsi::sph::ChFsiFluidSystemSPH::ElasticMaterialProperties soil_props;
    };

    /// Problem geometry (fluid domain, container, computational domain).
    struct ProblemGeometry {
        ProblemGeometry();
        void PrintInfo();

        std::unique_ptr<BoxDomain> fluid_domain_cartesian;
        std::unique_ptr<AnnulusDomain> fluid_domain_cylindrical;
        std::unique_ptr<BoxDomain> container_cartesian;
        std::unique_ptr<AnnulusDomain> container_cylindrical;

        std::unique_ptr<ComputationalDomain> computational_domain;
    };

    /// Wave tank settings.
    struct Wavetank {
        Wavetank();
        void PrintInfo();

        fsi::sph::ChFsiProblemWavetank::WavemakerType type;
        BoxDomain container;
        double depth;
        bool end_wall;

        std::shared_ptr<ChFunctionInterp> profile;
        std::shared_ptr<ChFunction> actuation;
        double actuation_delay;
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

    /// Simulation and run-time visualization parameters.
    struct SimParams {
        SimParams();
        void PrintInfo();

        double time_step;
        double end_time;

        ChVector3d gravity;

        fsi::sph::ChFsiFluidSystemSPH::SPHParameters sph;
        VisParams visualization;
    };

    /// Output database.
    struct OutputData {
        //// TODO
    };

  private:  // ---- Functions
    static GeometryType ReadGeometryType(const YAML::Node& a);
    static fsi::sph::PhysicsProblem ReadPhysicsProblemType(const YAML::Node& a);
    static fsi::sph::ChFsiProblemWavetank::WavemakerType ReadWavetankType(const YAML::Node& a);
    static fsi::sph::EosType ReadEosType(const YAML::Node& a);
    static fsi::sph::KernelType ReadKernelType(const YAML::Node& a);
    static fsi::sph::IntegrationScheme ReadIntegrationScheme(const YAML::Node& a);
    static fsi::sph::BoundaryMethod ReadBoundaryMethod(const YAML::Node& a);
    static fsi::sph::ShiftingMethod ReadShiftingMethod(const YAML::Node& a);
    static fsi::sph::ViscosityMethod ReadViscosityMethod(const YAML::Node& a);

    static int ReadWallFlagsCartesian(const YAML::Node& a);
    static int ReadWallFlagsCylindrical(const YAML::Node& a);

    static fsi::sph::BCType ReadBoundaryConditionType(const YAML::Node& a);

    static ParticleColoringType ReadParticleColoringType(const YAML::Node& a);
    static ChColormap::Type ReadColorMapType(const YAML::Node& a);
#ifdef CHRONO_VSG
    static fsi::sph::MarkerPlanesVisibilityCallback::Mode ReadVisibilityMode(const YAML::Node& a);
#endif

  private:  // ---- Member variables
    GeometryType m_geometry_type;

    MaterialProperties m_material;  ///< material properties
    ProblemGeometry m_geometry;     ///< fluid parameters
    Wavetank m_wavetank;            ///< wave tank settings
    SimParams m_sim;                ///< simulation parameters

    std::shared_ptr<fsi::sph::ChFsiProblemSPH> m_fsi_problem;  ///< underlying FSI problem

    bool m_has_wavetank;

    bool m_depth_based_pressure;
    double m_zero_height;
    bool m_initial_velocity;
    ChVector3d m_velocity;

    OutputData m_output_data;  ///< output data

    bool m_sim_loaded;    ///< YAML simulation file loaded
    bool m_model_loaded;  ///< YAML model file loaded
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
