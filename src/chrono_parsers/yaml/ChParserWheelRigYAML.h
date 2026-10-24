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

#ifndef CH_PARSER_WHEEL_RIG_YAML_H
#define CH_PARSER_WHEEL_RIG_YAML_H

#include <functional>
#include <map>
#include <string>

#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
    #ifdef CHRONO_CRM
        #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
    #endif
#endif

namespace chrono {
namespace parsers {

/// @addtogroup parsers_module
/// @{

/// Parser for YAML specification files for Chrono::Vehicle single-wheel test rig simulations (ChWheelTestRig).
/// A WHEEL_RIG simulation specification file references a rig mechanism file (wheel assembly, rig properties, and MBS
/// solver) and a terrain file (terrain type, patch, parameters, and, for CRM terrain, the SPH solver), and specifies
/// the test scenario and the wheel-terrain coupling settings.
/// The wheel assembly is either a Chrono::Vehicle wheel, tire, and (optional) suspension specified through JSON files
/// (VEHICLE model type), or a custom wheel assembly provided by the application (CUSTOM model type) through a factory
/// registered with RegisterWheelAssembly.
class ChApiParsers ChParserWheelRigYAML : public ChParserMbsYAML {
  public:
    /// Factory for a custom wheel assembly.
    /// The factory is invoked with the containing system and the `parameters` node of the rig model specification (an
    /// undefined node if no parameters are specified) and must return a wheel assembly constructed in that system.
    using WheelAssemblyFactory = std::function<std::shared_ptr<vehicle::ChWheelTestRig::WheelAssembly>(ChSystem& sys, const YAML::Node& parameters)>;

    ChParserWheelRigYAML(const std::string& yaml_filename, bool verbose = false);
    ~ChParserWheelRigYAML();

    /// Register a factory for custom wheel assemblies with the given name.
    /// A rig model of CUSTOM type uses the factory registered under the name specified as `model.assembly`.
    /// Factories must be registered before calling CreateRig.
    void RegisterWheelAssembly(const std::string& name, WheelAssemblyFactory factory);

    /// Return true if the rig model specifies a custom wheel assembly.
    bool IsCustomAssembly() const { return m_custom; }

    /// Return the name of the custom wheel assembly (empty for a VEHICLE model type).
    const std::string& GetAssemblyName() const { return m_assembly_name; }

    /// Return the full path of a data file specified relative to the data path of the rig model specification.
    /// This can be used by custom wheel assembly factories to locate their data files.
    std::string GetDataFile(const std::string& filename) const { return m_file_handler.GetFilename(filename); }

    /// Load the specified WHEEL_RIG simulation specification file (and the rig and terrain files it references).
    virtual void LoadFile(const std::string& yaml_filename) override;

    /// Create the single-wheel test rig in the given system from the cached specification, then initialize it.
    /// The system should be created with CreateSystem().
    std::shared_ptr<vehicle::ChWheelTestRig> CreateRig(ChSystem& sys);

    /// Return the test rig (null if not yet created).
    std::shared_ptr<vehicle::ChWheelTestRig> GetRig() const { return m_rig; }

    /// Return the terrain type.
    vehicle::ChWheelTestRig::TerrainType GetTerrainType() const { return m_terrain_type; }

    /// Return the rig step, i.e., the interval for each call to ChWheelTestRig::Advance.
    /// With CRM terrain, this is the FSI coupling step.
    double GetRigStep() const { return m_rig_step > 0 ? m_rig_step : GetTimestep(); }

    /// Return true if the simulation should end when the wheel reaches the end of the terrain patch.
    /// See ChWheelTestRig::PatchEndReached.
    bool StopAtPatchEnd() const { return m_stop_at_patch_end; }

    /// Return true if generating output, i.e., writing an output database or recording data for plots.
    virtual bool OutputEnabled() const override { return ChParserYAML::OutputEnabled() || PlotsEnabled(); }

    /// Return true if plots were requested (in the `plots` section of the simulation specification).
    bool PlotsEnabled() const { return !m_figures.empty(); }

    /// Write the rig output channels (see ChWheelTestRig::GetOutputChannels) at the current time and record them for
    /// plotting (if plots were requested).
    /// The output directory must be set (SetOutputDir) and the rig created (CreateRig) before generating output.
    virtual void WriteOutput(int frame, double time) override;

    /// Generate the requested plots from the data recorded during the simulation.
    /// For each figure, a Gnuplot script is written to the output directory and executed, producing an image file or
    /// an interactive window. This function must be called once, at the end of the simulation; it does nothing if no
    /// plots were requested. Plotting requires the Chrono::Postprocess module and Gnuplot.
    void GeneratePlots();

    /// Return the camera location relative to the wheel hub position.
    const ChVector3d& GetCameraOffset() const { return m_camera_offset; }

    /// Return the camera look-at point relative to the wheel hub position.
    const ChVector3d& GetCameraTargetOffset() const { return m_camera_target_offset; }

#ifdef CHRONO_VSG
    /// Return a VSG plugin for rendering the SPH particles and BCE markers of a CRM terrain.
    /// Returns null for other terrain types or if the rig was not yet created.
    std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> GetVisualizationPlugin() const;
#endif

  private:
    void LoadRigFile(const std::string& filename);
    void LoadTerrainFile(const std::string& filename);
    void LoadTerrainSPH(const std::string& filename);
    void LoadTestData(const YAML::Node& yaml);
    void LoadCouplingData(const YAML::Node& yaml);

    /// Read a function of time with angle values, converting them to radians if necessary.
    std::shared_ptr<ChFunction> ReadAngleFunction(const YAML::Node& a) const;

    static vehicle::ChWheelTestRig::Mode ReadMode(const YAML::Node& a);
    static vehicle::ChWheelTestRig::TerrainType ReadTerrainType(const YAML::Node& a);
    static vehicle::ChTire::CollisionType ReadTireCollisionType(const YAML::Node& a);

    void LoadPlotData(const YAML::Node& yaml);

    /// Plot output type.
    enum class PlotOutput { WINDOW, PNG, PDF };

    /// Figure specification: one or more channels as functions of time or of another channel.
    struct Figure {
        std::string x;               ///< abscissa channel (empty for time)
        std::vector<std::string> y;  ///< ordinate channels
        std::string title;           ///< figure title (optional)
        bool set_range;              ///< true if an ordinate range was specified
        double y_min;                ///< ordinate range (lower bound)
        double y_max;                ///< ordinate range (upper bound)
    };

    enum class SteeringInput { NONE, ANGLE, ACTUATOR };
    enum class ActiveDomain { NONE, AUTO, BOX };

    // Rig mechanism
    bool m_custom;
    std::string m_assembly_name;
    YAML::Node m_assembly_params;
    std::map<std::string, WheelAssemblyFactory> m_factories;
    std::string m_wheel_json;
    std::string m_tire_json;
    std::string m_suspension_json;
    bool m_set_collision_type;
    vehicle::ChTire::CollisionType m_collision_type;
    double m_normal_load;
    double m_camber_angle;

    // Terrain
    vehicle::ChWheelTestRig::TerrainType m_terrain_type;
    vehicle::ChWheelTestRig::TerrainPatchSize m_patch_size;
    vehicle::ChWheelTestRig::TerrainParamsRigid m_params_rigid;
    vehicle::ChWheelTestRig::TerrainParamsSCM m_params_scm;
    vehicle::ChWheelTestRig::TerrainParamsGranular m_params_granular;
#ifdef CHRONO_CRM
    vehicle::ChWheelTestRig::TerrainParamsCRM m_params_crm;
    #ifdef CHRONO_VSG
    fsi::sph::ChSphVisualizationVSG::Settings m_vis_sph_settings;
    #endif
#endif

    // Test scenario
    bool m_test_degrees;
    vehicle::ChWheelTestRig::Mode m_mode;
    double m_drop_speed;
    double m_time_delay;
    std::shared_ptr<ChFunction> m_long_speed_fun;
    std::shared_ptr<ChFunction> m_ang_speed_fun;
    std::shared_ptr<ChFunction> m_slip_angle_fun;
    SteeringInput m_steering_input;
    std::shared_ptr<ChFunction> m_steering_fun;
    bool m_steering_feedback;
    double m_steering_time_constant;
    bool m_long_slip;
    double m_long_slip_value;
    double m_long_slip_base_speed;

    // Coupling
    ActiveDomain m_active_domain;
    ChVector3d m_active_domain_size;

    // Simulation and visualization
    double m_rig_step;
    bool m_stop_at_patch_end;
    ChVector3d m_camera_offset;
    ChVector3d m_camera_target_offset;

    // Plots
    std::vector<Figure> m_figures;                 ///< requested figures
    PlotOutput m_plot_output;                      ///< plot output type
    bool m_plot_measuring_only;                    ///< plot only while measurements are enabled
    std::vector<std::string> m_plot_channels;      ///< names of recorded channels
    std::vector<std::string> m_plot_units;         ///< units of recorded channels
    std::vector<double> m_plot_time;               ///< recorded times
    std::vector<std::vector<double>> m_plot_data;  ///< recorded channel values (one vector per channel)

    std::shared_ptr<vehicle::ChWheelTestRig> m_rig;
};

/// @} parsers_module

}  // namespace parsers
}  // namespace chrono

#endif
