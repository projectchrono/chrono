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

#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification files for Chrono::SPH models and simulations.
/// The parser caches model information and simulation settings from a YAML input file and then allows populating an FSI
/// Chrono::SPH system and setting solver and simulation parameters.
class ChApiParsers ChParserSphYAML : public ChParserCfdYAML {
  public:
    ChParserSphYAML(const std::string& yaml_filename, bool verbose = false);
    ~ChParserSphYAML();

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
    void LoadSolverData(const YAML::Node& yaml);

    double GetTimestep() const { return m_sim.time_step; }
    double GetEndtime() const { return m_sim.end_time; }

    // --------------

    /// Create and return a Chrono FSI problem configured from cached model and simulation parameters.
    /// By default, the Chrono FSI problem is initialized (with no associated MBS system). If a system is attached after
    /// creation, the caller must create the FSI problem with initialize=false, attach an MBS to the problem with
    /// ChFsiProblemSPH::AttachMultibodySystem, and then explicitly invoke ChFsiProblemSPH::Initialize().
    std::shared_ptr<fsi::sph::ChFsiProblemSPH> CreateFsiProblemSPH(bool initialize = true);

    /// Access the underlying FSI system.
    virtual std::shared_ptr<fsi::ChFsiSystem> GetFsiSystem() override { return m_fsi_problem->GetFsiSystemSPH(); }

    /// Access the underlying fluid system.
    virtual std::shared_ptr<fsi::ChFsiFluidSystem> GetFluidSystem() override { return m_fsi_problem->GetFluidSystemSPH(); }

    // --------------

#ifdef CHRONO_VSG
    const fsi::sph::ChSphVisualizationVSG::Settings& GetSphVisualizationSettings() const;
    const fsi::sph::ChFsiFluidSystemSPH::SplashsurfParameters& GetSplashsurfParameters() const;
    bool UseSplashurf() const { return m_visSPH_settings.use_splashsurf; }
    virtual std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const override;
#endif

    // --------------

    /// Write simulation output results at the current time.
    virtual void WriteOutput(int frame, double time) override;

  private:
    enum class GeometryType { CARTESIAN, CYLINDRICAL };

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
        void PrintInfo() const;

        fsi::sph::PhysicsProblem physics_problem;
        fsi::sph::ChFsiFluidSystemSPH::FluidProperties fluid_props;
        fsi::sph::ChFsiFluidSystemSPH::ElasticMaterialProperties soil_props;
    };

    /// Problem geometry (fluid domain, container, computational domain).
    struct ProblemGeometry {
        ProblemGeometry();
        void PrintInfo() const;

        std::unique_ptr<BoxDomain> fluid_domain_cartesian;
        std::unique_ptr<AnnulusDomain> fluid_domain_cylindrical;
        std::unique_ptr<BoxDomain> container_cartesian;
        std::unique_ptr<AnnulusDomain> container_cylindrical;

        std::unique_ptr<ComputationalDomain> computational_domain;
    };

    /// Wave tank settings.
    struct Wavetank {
        Wavetank();
        void PrintInfo() const;

        fsi::sph::ChFsiProblemWavetank::WavemakerType type;
        BoxDomain container;
        double depth;
        bool end_wall;

        std::shared_ptr<ChFunctionInterp> profile;
        std::shared_ptr<ChFunction> actuation;
        double actuation_delay;
    };

    /// Simulation and run-time visualization parameters.
    struct SimParams {
        SimParams();
        void PrintInfo() const;

        double end_time;
        ChVector3d gravity;

        double time_step;
        fsi::sph::ChFsiFluidSystemSPH::SPHParameters sph;
    };

    /// Output database.
    struct OutputData {
        //// TODO
    };

  private:
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

  private:
    GeometryType m_geometry_type;  ///< geometry coordinate system (Cartesian or cylindrical)

    MaterialProperties m_material;  ///< material properties
    ProblemGeometry m_geometry;     ///< fluid parameters
    Wavetank m_wavetank;            ///< wave tank settings
    SimParams m_sim;                ///< simulation settings
#ifdef CHRONO_VSG
    fsi::sph::ChSphVisualizationVSG::Settings m_visSPH_settings;  ///< SPH visualization settings
#endif

    std::shared_ptr<fsi::sph::ChFsiProblemSPH> m_fsi_problem;  ///< underlying FSI problem

    bool m_has_wavetank;

    bool m_depth_based_pressure;
    double m_zero_height;
    bool m_initial_velocity;
    ChVector3d m_velocity;

    OutputData m_output_data;  ///< output data

    bool m_loaded;         ///< YAML simulation file loaded
    bool m_solver_loaded;  ///< YAML solver file loaded
    bool m_model_loaded;   ///< YAML model file loaded
};

/// @} parsers_module

}  // end namespace parsers
}  // namespace chrono

#endif
