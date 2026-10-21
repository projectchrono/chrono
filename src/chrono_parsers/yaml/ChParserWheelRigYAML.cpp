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

#include <filesystem>

#include "chrono/functions/ChFunctionConst.h"
#include "chrono/functions/ChFunctionOperator.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/utils/ChVehicleUtilsJSON.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

#include "chrono_parsers/yaml/ChParserWheelRigYAML.h"

#ifdef CHRONO_CRM
    #include "chrono_vehicle/terrain/CRMTerrain.h"
    #include "chrono_parsers/yaml/ChParserSphYAML.h"
#endif

#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChGnuPlot.h"
#endif

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

// -----------------------------------------------------------------------------

// Return the full path of a file specified relative to the directory of a reference file.
static std::string GetRelativeFile(const std::string& reference_file, const std::string& filename) {
    auto dir = std::filesystem::path(reference_file).parent_path();
    auto path = dir.empty() ? std::filesystem::path(filename) : dir / filename;
    if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
        cerr << "Error: file '" << path.string() << "' not found." << endl;
        throw std::runtime_error("File not found");
    }
    return path.string();
}

// Load a YAML file and check its Chrono version.
static YAML::Node LoadYamlFile(const std::string& filename) {
    auto yaml = YAML::LoadFile(filename);
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);
    return yaml;
}

// -----------------------------------------------------------------------------

ChParserWheelRigYAML::ChParserWheelRigYAML(const std::string& yaml_filename, bool verbose)
    : ChParserMbsYAML(verbose),
      m_custom(false),
      m_set_collision_type(false),
      m_collision_type(vehicle::ChTire::CollisionType::SINGLE_POINT),
      m_normal_load(1000),
      m_camber_angle(0),
      m_terrain_type(vehicle::ChWheelTestRig::TerrainType::NONE),
      m_test_degrees(true),
      m_mode(vehicle::ChWheelTestRig::Mode::TEST),
      m_drop_speed(0.1),
      m_time_delay(0),
      m_steering_input(SteeringInput::NONE),
      m_steering_feedback(false),
      m_steering_time_constant(0.05),
      m_long_slip(false),
      m_long_slip_value(0),
      m_long_slip_base_speed(1),
      m_active_domain(ActiveDomain::NONE),
      m_active_domain_size(VNULL),
      m_rig_step(-1),
      m_stop_at_patch_end(false),
      m_camera_offset(2.0, 2.5, 1.0),
      m_camera_target_offset(0, 0.25, -0.25),
      m_plot_output(PlotOutput::PNG),
      m_plot_measuring_only(true) {
    LoadFile(yaml_filename);
}

ChParserWheelRigYAML::~ChParserWheelRigYAML() {}

// -----------------------------------------------------------------------------

void ChParserWheelRigYAML::LoadFile(const std::string& yaml_filename) {
    auto yaml = LoadYamlFile(yaml_filename);

    // Check the YAML file is of type "WHEEL_RIG"
    ChAssertAlways(yaml["type"]);
    auto type = ReadYamlFileType(yaml["type"]);
    ChAssertAlways(type == ChParserYAML::YamlFileType::WHEEL_RIG);

    // Load simulation, output, and run-time visualization data
    LoadSimData(yaml);
    if (yaml["simulation"]["time_step"])
        m_rig_step = yaml["simulation"]["time_step"].as<double>();
    if (yaml["simulation"]["stop_at_patch_end"])
        m_stop_at_patch_end = yaml["simulation"]["stop_at_patch_end"].as<bool>();
    if (yaml["visualization"] && yaml["visualization"]["camera"]) {
        auto camera = yaml["visualization"]["camera"];
        if (camera["offset"])
            m_camera_offset = ReadVector(camera["offset"]);
        if (camera["target_offset"])
            m_camera_target_offset = ReadVector(camera["target_offset"]);
    }

    // The test rig assumes a vertical Z axis
    const auto& gravity = GetGravitationalAcceleration();
    if (std::abs(gravity.x()) > 1e-12 || std::abs(gravity.y()) > 1e-12 || gravity.z() > 0) {
        cerr << "Error: the gravitational acceleration must be of the form [0, 0, -g]." << endl;
        throw std::runtime_error("Incorrect gravitational acceleration");
    }

    // Load the rig mechanism and terrain files
    ChAssertAlways(yaml["rig"]);
    ChAssertAlways(yaml["terrain"]);
    LoadRigFile(GetRelativeFile(yaml_filename, yaml["rig"].as<std::string>()));
    LoadTerrainFile(GetRelativeFile(yaml_filename, yaml["terrain"].as<std::string>()));

    // Load the test scenario and coupling settings
    ChAssertAlways(yaml["test"]);
    LoadTestData(yaml["test"]);
    if (yaml["coupling"])
        LoadCouplingData(yaml["coupling"]);

    // Load plot specifications
    if (yaml["plots"])
        LoadPlotData(yaml["plots"]);

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\nWheel test rig: " << m_name << endl;
        if (m_custom) {
            cout << "  custom wheel assembly: " << m_assembly_name << endl;
        } else {
            cout << "  wheel:      " << m_wheel_json << endl;
            cout << "  tire:       " << m_tire_json << endl;
            cout << "  suspension: " << (m_suspension_json.empty() ? "none" : m_suspension_json) << endl;
        }
        cout << "  normal load: " << m_normal_load << endl;
        cout << "  rig step:    " << GetRigStep() << endl;
        cout << "  plots:       " << m_figures.size() << endl;
        cout << endl;
        m_vis_settings.PrintInfo();
    }

    m_loaded = true;
}

void ChParserWheelRigYAML::LoadRigFile(const std::string& filename) {
    if (m_verbose)
        cout << "\nLoading wheel test rig mechanism from: " << filename << endl;

    auto yaml = LoadYamlFile(filename);

    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    if (model["name"])
        m_name = model["name"].as<std::string>();

    bool use_degrees = true;
    if (model["angle_degrees"])
        use_degrees = model["angle_degrees"].as<bool>();

    // Data files are located relative to the rig file
    m_file_handler.SetReferenceDirectory(filename);
    m_file_handler.Read(model);

    // Wheel assembly type (VEHICLE: Chrono::Vehicle JSON specification files; CUSTOM: provided by the application)
    if (model["type"]) {
        auto type = ChToUpper(model["type"].as<std::string>());
        if (type == "CUSTOM") {
            m_custom = true;
        } else if (type != "VEHICLE") {
            cerr << "Error: unknown rig model type '" << model["type"].as<std::string>() << "'." << endl;
            throw std::runtime_error("Unknown rig model type");
        }
    }

    if (m_custom) {
        ChAssertAlways(model["assembly"]);
        m_assembly_name = model["assembly"].as<std::string>();
        if (model["parameters"])
            m_assembly_params = model["parameters"];
    } else {
        ChAssertAlways(model["wheel_json"]);
        ChAssertAlways(model["tire_json"]);
        m_wheel_json = m_file_handler.GetFilename(model["wheel_json"].as<std::string>());
        m_tire_json = m_file_handler.GetFilename(model["tire_json"].as<std::string>());
        if (model["suspension_json"])
            m_suspension_json = m_file_handler.GetFilename(model["suspension_json"].as<std::string>());

        if (model["tire"] && model["tire"]["collision_type"]) {
            m_set_collision_type = true;
            m_collision_type = ReadTireCollisionType(model["tire"]["collision_type"]);
        }
    }

    if (yaml["rig"]) {
        auto rig = yaml["rig"];
        if (rig["normal_load"])
            m_normal_load = rig["normal_load"].as<double>();
        if (rig["camber_angle"])
            m_camber_angle = rig["camber_angle"].as<double>() * (use_degrees ? CH_DEG_TO_RAD : 1.0);
    }

    // MBS solver (relative to the rig file)
    ChAssertAlways(yaml["solver"]);
    auto solver_filename = GetRelativeFile(filename, yaml["solver"].as<std::string>());
    if (m_verbose)
        cout << "Loading MBS solver specification from: " << solver_filename << endl;
    LoadSolverData(LoadYamlFile(solver_filename));
}

void ChParserWheelRigYAML::LoadTerrainFile(const std::string& filename) {
    if (m_verbose)
        cout << "Loading terrain specification from: " << filename << endl;

    auto yaml = LoadYamlFile(filename);

    ChAssertAlways(yaml["type"]);
    m_terrain_type = ReadTerrainType(yaml["type"]);

    ChAssertAlways(yaml["patch"]);
    auto patch = yaml["patch"];
    ChAssertAlways(patch["length"]);
    ChAssertAlways(patch["width"]);
    m_patch_size.length = patch["length"].as<double>();
    m_patch_size.width = patch["width"].as<double>();
    if (patch["depth"])
        m_patch_size.depth = patch["depth"].as<double>();

    switch (m_terrain_type) {
        case vehicle::ChWheelTestRig::TerrainType::RIGID:
            if (yaml["rigid"]) {
                auto a = yaml["rigid"];
                if (a["friction"])
                    m_params_rigid.mu = a["friction"].as<float>();
                if (a["restitution"])
                    m_params_rigid.cr = a["restitution"].as<float>();
                if (a["Young_modulus"])
                    m_params_rigid.Y = a["Young_modulus"].as<float>();
            }
            break;
        case vehicle::ChWheelTestRig::TerrainType::SCM:
            if (yaml["scm"]) {
                auto a = yaml["scm"];
                if (a["Bekker_Kphi"])
                    m_params_scm.Bekker_Kphi = a["Bekker_Kphi"].as<double>();
                if (a["Bekker_Kc"])
                    m_params_scm.Bekker_Kc = a["Bekker_Kc"].as<double>();
                if (a["Bekker_n"])
                    m_params_scm.Bekker_n = a["Bekker_n"].as<double>();
                if (a["Mohr_cohesion"])
                    m_params_scm.Mohr_cohesion = a["Mohr_cohesion"].as<double>();
                if (a["Mohr_friction"])
                    m_params_scm.Mohr_friction = a["Mohr_friction"].as<double>();
                if (a["Janosi_shear"])
                    m_params_scm.Janosi_shear = a["Janosi_shear"].as<double>();
                if (a["grid_spacing"])
                    m_params_scm.grid_spacing = a["grid_spacing"].as<double>();
                if (a["visualize_mesh"])
                    m_params_scm.vis_mesh = a["visualize_mesh"].as<bool>();
            }
            break;
        case vehicle::ChWheelTestRig::TerrainType::GRANULAR:
            if (yaml["granular"]) {
                auto a = yaml["granular"];
                if (a["radius"])
                    m_params_granular.radius = a["radius"].as<double>();
                if (a["density"])
                    m_params_granular.density = a["density"].as<double>();
                if (a["friction"])
                    m_params_granular.friction = a["friction"].as<double>();
                if (a["cohesion"])
                    m_params_granular.cohesion = a["cohesion"].as<double>();
                if (a["Young_modulus"])
                    m_params_granular.Young_modulus = a["Young_modulus"].as<double>();
            }
            break;
        case vehicle::ChWheelTestRig::TerrainType::CRM: {
#ifdef CHRONO_CRM
            ChAssertAlways(yaml["crm"]);
            auto a = yaml["crm"];

            // Start from the generic SPH defaults (fsisph_solver.schema), not the test rig CRM defaults
            m_params_crm.sph_params = fsi::sph::ChFsiFluidSystemSPH::SPHParameters();
            m_params_crm.mat_props = fsi::sph::ChFsiFluidSystemSPH::SoilProperties();

            if (a["soil_properties"])
                ChParserSphYAML::ReadSoilProperties(a["soil_properties"], m_params_crm.mat_props);

            ChAssertAlways(a["solver"]);
            auto solver_filename = GetRelativeFile(filename, a["solver"].as<std::string>());
            if (m_verbose)
                cout << "Loading SPH solver specification from: " << solver_filename << endl;
            ChParserSphYAML::ReadSPHParameters(LoadYamlFile(solver_filename), m_params_crm.sph_params, m_params_crm.step_cfd);

    #ifdef CHRONO_VSG
            if (a["visualization"])
                m_vis_sph_settings = fsi::sph::ChSphVisualizationVSG::Settings::Read(a["visualization"]);
    #endif
#else
            cerr << "Error: CRM terrain requires Chrono::Vehicle with CRM support." << endl;
            throw std::runtime_error("CRM terrain not supported");
#endif
            break;
        }
        default:
            break;
    }
}

void ChParserWheelRigYAML::LoadTestData(const YAML::Node& a) {
    if (a["angle_degrees"])
        m_test_degrees = a["angle_degrees"].as<bool>();

    ChAssertAlways(a["mode"]);
    m_mode = ReadMode(a["mode"]);
    if (a["drop_speed"])
        m_drop_speed = a["drop_speed"].as<double>();
    if (a["time_delay"])
        m_time_delay = a["time_delay"].as<double>();

    if (a["longitudinal_speed"])
        m_long_speed_fun = ReadFunction(a["longitudinal_speed"], m_test_degrees);
    if (a["angular_speed"])
        m_ang_speed_fun = ReadFunction(a["angular_speed"], m_test_degrees);
    if (a["slip_angle"])
        m_slip_angle_fun = ReadAngleFunction(a["slip_angle"]);

    if (a["steering"]) {
        auto s = a["steering"];
        ChAssertAlways(s["input"]);
        ChAssertAlways(s["function"]);
        auto input = ChToUpper(s["input"].as<std::string>());
        if (input == "ANGLE") {
            m_steering_input = SteeringInput::ANGLE;
            m_steering_fun = ReadAngleFunction(s["function"]);
        } else if (input == "ACTUATOR") {
            m_steering_input = SteeringInput::ACTUATOR;
            m_steering_fun = ReadFunction(s["function"], m_test_degrees);
        } else {
            cerr << "Error: unknown steering input '" << s["input"].as<std::string>() << "'." << endl;
            throw std::runtime_error("Unknown steering input");
        }
        if (s["feedback"]) {
            if (s["feedback"]["enabled"])
                m_steering_feedback = s["feedback"]["enabled"].as<bool>();
            if (s["feedback"]["time_constant"])
                m_steering_time_constant = s["feedback"]["time_constant"].as<double>();
        }
    }

    if (a["longitudinal_slip"]) {
        auto s = a["longitudinal_slip"];
        ChAssertAlways(s["value"]);
        m_long_slip = true;
        m_long_slip_value = s["value"].as<double>();
        if (s["base_speed"])
            m_long_slip_base_speed = s["base_speed"].as<double>();
    }
}

void ChParserWheelRigYAML::LoadCouplingData(const YAML::Node& a) {
    if (a["crm"] && a["crm"]["active_domain"]) {
        auto d = a["crm"]["active_domain"];
        ChAssertAlways(d["type"]);
        auto type = ChToUpper(d["type"].as<std::string>());
        if (type == "AUTO") {
            m_active_domain = ActiveDomain::AUTO;
        } else if (type == "BOX") {
            m_active_domain = ActiveDomain::BOX;
            ChAssertAlways(d["size"]);
            m_active_domain_size = ReadVector(d["size"]);
        } else {
            m_active_domain = ActiveDomain::NONE;
        }
    }
}

std::shared_ptr<ChFunction> ChParserWheelRigYAML::ReadAngleFunction(const YAML::Node& a) const {
    auto fun = ReadFunction(a, m_test_degrees);
    if (!m_test_degrees)
        return fun;

    auto scaled = chrono_types::make_shared<ChFunctionOperator>();
    scaled->SetOperationType(ChFunctionOperator::MUL);
    scaled->SetFirstOperandFunction(fun);
    scaled->SetSecondOperandFunction(chrono_types::make_shared<ChFunctionConst>(CH_DEG_TO_RAD));
    return scaled;
}

// -----------------------------------------------------------------------------

void ChParserWheelRigYAML::WriteOutput(int frame, double time) {
    if (!m_rig)
        return;

    auto channels = m_rig->GetOutputChannels();

    // Write to the output database (created if needed; null if no output database or format not available)
    ChParserYAML::WriteOutput(frame, time);
    if (m_output_db)
        m_output_db->Write(frame, time, {}, channels);

    // Record channels for plotting
    if (PlotsEnabled()) {
        if (m_plot_channels.empty()) {
            for (const auto& channel : channels) {
                m_plot_channels.push_back(channel.name);
                m_plot_units.push_back(channel.units);
            }
            m_plot_data.resize(channels.size());
        }
        m_plot_time.push_back(time);
        for (size_t i = 0; i < channels.size(); i++)
            m_plot_data[i].push_back(channels[i].value);
    }
}

void ChParserWheelRigYAML::LoadPlotData(const YAML::Node& a) {
    if (a["output"]) {
        auto output = ChToUpper(a["output"].as<std::string>());
        if (output == "WINDOW")
            m_plot_output = PlotOutput::WINDOW;
        else if (output == "PNG")
            m_plot_output = PlotOutput::PNG;
        else if (output == "PDF")
            m_plot_output = PlotOutput::PDF;
        else {
            cerr << "Error: unknown plot output '" << a["output"].as<std::string>() << "'." << endl;
            throw std::runtime_error("Unknown plot output");
        }
    }
    if (a["measuring_only"])
        m_plot_measuring_only = a["measuring_only"].as<bool>();

    if (!a["figures"])
        return;
    for (const auto& f : a["figures"]) {
        Figure figure;
        figure.set_range = false;
        if (f.IsScalar()) {
            // Shorthand: a single channel as a function of time
            figure.y.push_back(f.as<std::string>());
        } else {
            ChAssertAlways(f["y"]);
            if (f["y"].IsSequence()) {
                for (const auto& y : f["y"])
                    figure.y.push_back(y.as<std::string>());
            } else {
                figure.y.push_back(f["y"].as<std::string>());
            }
            if (f["x"])
                figure.x = f["x"].as<std::string>();
            if (f["title"])
                figure.title = f["title"].as<std::string>();
            if (f["y_range"]) {
                ChAssertAlways(f["y_range"].IsSequence() && f["y_range"].size() == 2);
                figure.set_range = true;
                figure.y_min = f["y_range"][0].as<double>();
                figure.y_max = f["y_range"][1].as<double>();
            }
        }
        m_figures.push_back(figure);
    }
}

void ChParserWheelRigYAML::GeneratePlots() {
    if (!PlotsEnabled())
        return;

#ifdef CHRONO_POSTPROCESS
    if (m_plot_time.empty()) {
        cerr << "Warning: no data recorded for plots." << endl;
        return;
    }

    // Index of a recorded channel (-1 if not found)
    auto find_channel = [this](const std::string& name) -> int {
        for (size_t i = 0; i < m_plot_channels.size(); i++) {
            if (m_plot_channels[i] == name)
                return (int)i;
        }
        return -1;
    };

    // Frames to plot (all, or only those with measurements enabled)
    std::vector<size_t> frames;
    int i_measuring = find_channel("measuring");
    for (size_t k = 0; k < m_plot_time.size(); k++) {
        if (!m_plot_measuring_only || i_measuring < 0 || m_plot_data[i_measuring][k] > 0)
            frames.push_back(k);
    }
    if (frames.empty()) {
        cerr << "Warning: no frames to plot (measurements were never enabled)." << endl;
        return;
    }

    auto select = [&frames](const std::vector<double>& values) {
        std::vector<double> selected;
        selected.reserve(frames.size());
        for (auto k : frames)
            selected.push_back(values[k]);
        return selected;
    };
    auto label = [this](int i) { return m_plot_channels[i] + " (" + m_plot_units[i] + ")"; };

    for (size_t f = 0; f < m_figures.size(); f++) {
        const auto& figure = m_figures[f];

        // Abscissa data and label
        std::vector<double> x_data;
        std::string x_label;
        if (figure.x.empty()) {
            x_data = select(m_plot_time);
            x_label = "time (s)";
        } else {
            int i = find_channel(figure.x);
            if (i < 0) {
                cerr << "Warning: unknown channel '" << figure.x << "' in plot " << f + 1 << ". Plot skipped." << endl;
                continue;
            }
            x_data = select(m_plot_data[i]);
            x_label = label(i);
        }

        // Ordinate channels
        std::vector<int> y_channels;
        for (const auto& name : figure.y) {
            int i = find_channel(name);
            if (i < 0)
                cerr << "Warning: unknown channel '" << name << "' in plot " << f + 1 << ". Channel skipped." << endl;
            else
                y_channels.push_back(i);
        }
        if (y_channels.empty())
            continue;

        std::string stem = m_output_dir + "/plot_" + std::to_string(f + 1);
        postprocess::ChGnuPlot gplot(stem + ".gpl");
        switch (m_plot_output) {
            case PlotOutput::PNG:
                gplot.SetPersist(false);
                gplot.OutputPNG(stem + ".png", 800, 600);
                break;
            case PlotOutput::PDF:
                gplot.SetPersist(false);
                gplot.OutputPDF(stem + ".pdf");
                break;
            case PlotOutput::WINDOW:
                break;
        }
        gplot.SetCommand("set termoption noenhanced");  // render channel names (with underscores) literally
        gplot.SetGrid();
        if (!figure.title.empty())
            gplot.SetTitle(figure.title);
        gplot.SetLabelX(x_label);
        gplot.SetLabelY(y_channels.size() == 1 ? label(y_channels[0]) : m_plot_units[y_channels[0]]);
        if (figure.set_range)
            gplot.SetRangeY(figure.y_min, figure.y_max);
        for (auto i : y_channels)
            gplot.Plot(x_data, select(m_plot_data[i]), m_plot_channels[i], " with lines");

        // The plot is generated when the ChGnuPlot object goes out of scope
    }
#else
    cerr << "Warning: plots require the Chrono::Postprocess module. No plots generated." << endl;
#endif
}

void ChParserWheelRigYAML::RegisterWheelAssembly(const std::string& name, WheelAssemblyFactory factory) {
    m_factories[name] = factory;
}

std::shared_ptr<vehicle::ChWheelTestRig> ChParserWheelRigYAML::CreateRig(ChSystem& sys) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\nCreate wheel test rig\n" << endl;
    }

    if (m_custom) {
        // Create the custom wheel assembly through the registered factory, then the test rig
        auto factory = m_factories.find(m_assembly_name);
        if (factory == m_factories.end()) {
            cerr << "Error: no factory registered for custom wheel assembly '" << m_assembly_name << "'." << endl;
            throw std::runtime_error("Unregistered custom wheel assembly");
        }
        auto assembly = factory->second(sys, m_assembly_params);
        ChAssertAlways(assembly);
        m_rig = chrono_types::make_shared<vehicle::ChWheelTestRig>(sys, assembly);
    } else {
        // Create the wheel assembly subsystems
        auto wheel = vehicle::ReadWheelJSON(m_wheel_json);
        auto tire = vehicle::ReadTireJSON(m_tire_json);
        if (m_set_collision_type)
            tire->SetCollisionType(m_collision_type);

        // Handling tires have no collision geometry and can only be used with rigid terrain
        if (std::dynamic_pointer_cast<vehicle::ChForceElementTire>(tire) && m_terrain_type != vehicle::ChWheelTestRig::TerrainType::RIGID) {
            cerr << "Error: handling tire models can only be used with rigid terrain." << endl;
            throw std::runtime_error("Incompatible tire and terrain");
        }

        // Create the test rig
        if (m_suspension_json.empty()) {
            m_rig = chrono_types::make_shared<vehicle::ChWheelTestRig>(sys, wheel, tire);
        } else {
            auto suspension = vehicle::ReadSuspensionJSON(m_suspension_json);
            m_rig = chrono_types::make_shared<vehicle::ChWheelTestRig>(sys, wheel, tire, suspension);
        }
    }

    // Rig properties
    m_rig->SetGravitationalAcceleration(-GetGravitationalAcceleration().z());
    m_rig->SetNormalLoad(m_normal_load);
    m_rig->SetCamberAngle(m_camber_angle);
    m_rig->SetStepsize(GetTimestep());
    m_rig->SetVisualizationType(GetVisualizationType());

    // Terrain
    switch (m_terrain_type) {
        case vehicle::ChWheelTestRig::TerrainType::RIGID:
            m_rig->SetTerrainRigid(m_patch_size, m_params_rigid);
            break;
        case vehicle::ChWheelTestRig::TerrainType::SCM:
            m_rig->SetTerrainSCM(m_patch_size, m_params_scm);
            break;
        case vehicle::ChWheelTestRig::TerrainType::GRANULAR:
            m_rig->SetTerrainGranular(m_patch_size, m_params_granular);
            break;
        case vehicle::ChWheelTestRig::TerrainType::CRM:
#ifdef CHRONO_CRM
            m_rig->SetTerrainCRM(m_patch_size, m_params_crm);
            if (m_active_domain == ActiveDomain::AUTO)
                m_rig->SetWheelActiveDomain();
            else if (m_active_domain == ActiveDomain::BOX)
                m_rig->SetWheelActiveDomain(ChAABB(-0.5 * m_active_domain_size, 0.5 * m_active_domain_size));
#endif
            break;
        default:
            break;
    }

    // Test scenario
    if (m_long_speed_fun)
        m_rig->SetLongSpeedFunction(m_long_speed_fun);
    if (m_ang_speed_fun)
        m_rig->SetAngSpeedFunction(m_ang_speed_fun);
    if (m_slip_angle_fun)
        m_rig->SetSlipAngleFunction(m_slip_angle_fun);
    switch (m_steering_input) {
        case SteeringInput::ANGLE:
            m_rig->SetSteeringAngleFunction(m_steering_fun);
            m_rig->EnableSteeringFeedback(m_steering_feedback, m_steering_time_constant);
            break;
        case SteeringInput::ACTUATOR:
            m_rig->SetSteeringActuatorFunction(m_steering_fun);
            break;
        default:
            break;
    }
    if (m_long_slip)
        m_rig->SetConstantLongitudinalSlip(m_long_slip_value, m_long_slip_base_speed);
    m_rig->SetTimeDelay(m_time_delay);

    m_rig->Initialize(m_mode, m_drop_speed);

    return m_rig;
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> ChParserWheelRigYAML::GetVisualizationPlugin() const {
    #ifdef CHRONO_CRM
    if (!m_rig || m_terrain_type != vehicle::ChWheelTestRig::TerrainType::CRM)
        return nullptr;

    auto terrain = std::static_pointer_cast<vehicle::CRMTerrain>(m_rig->GetTerrain());
    auto vis = chrono_types::make_shared<fsi::sph::ChSphVisualizationVSG>(terrain->GetFsiSystemSPH().get());
    vis->EnableFluidMarkers(m_vis_sph_settings.sph_markers);
    vis->EnableBoundaryMarkers(m_vis_sph_settings.bndry_bce_markers);
    vis->EnableRigidBodyMarkers(m_vis_sph_settings.rigid_bce_markers);
    if (m_vis_sph_settings.color_callback)
        vis->SetSPHColorCallback(m_vis_sph_settings.color_callback, m_vis_sph_settings.colormap);
    if (m_vis_sph_settings.visibility_callback_sph)
        vis->SetSPHVisibilityCallback(m_vis_sph_settings.visibility_callback_sph);
    if (m_vis_sph_settings.visibility_callback_bce)
        vis->SetBCEVisibilityCallback(m_vis_sph_settings.visibility_callback_bce);
    return vis;
    #else
    return nullptr;
    #endif
}
#endif

// -----------------------------------------------------------------------------

vehicle::ChWheelTestRig::Mode ChParserWheelRigYAML::ReadMode(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "SUSPEND")
        return vehicle::ChWheelTestRig::Mode::SUSPEND;
    if (val == "DROP")
        return vehicle::ChWheelTestRig::Mode::DROP;
    if (val == "TEST")
        return vehicle::ChWheelTestRig::Mode::TEST;
    cerr << "Error: unknown test rig mode '" << a.as<std::string>() << "'." << endl;
    throw std::runtime_error("Unknown test rig mode");
}

vehicle::ChWheelTestRig::TerrainType ChParserWheelRigYAML::ReadTerrainType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "RIGID")
        return vehicle::ChWheelTestRig::TerrainType::RIGID;
    if (val == "SCM")
        return vehicle::ChWheelTestRig::TerrainType::SCM;
    if (val == "GRANULAR")
        return vehicle::ChWheelTestRig::TerrainType::GRANULAR;
    if (val == "CRM")
        return vehicle::ChWheelTestRig::TerrainType::CRM;
    cerr << "Error: unknown terrain type '" << a.as<std::string>() << "'." << endl;
    throw std::runtime_error("Unknown terrain type");
}

vehicle::ChTire::CollisionType ChParserWheelRigYAML::ReadTireCollisionType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "SINGLE_POINT")
        return vehicle::ChTire::CollisionType::SINGLE_POINT;
    if (val == "FOUR_POINTS")
        return vehicle::ChTire::CollisionType::FOUR_POINTS;
    if (val == "ENVELOPE")
        return vehicle::ChTire::CollisionType::ENVELOPE;
    cerr << "Error: unknown tire collision type '" << a.as<std::string>() << "'." << endl;
    throw std::runtime_error("Unknown tire collision type");
}

}  // namespace parsers
}  // namespace chrono
