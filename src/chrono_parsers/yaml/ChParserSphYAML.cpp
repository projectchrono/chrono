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

//// TODO
//// - associate FSI flexible solids
//// - output

#include <algorithm>
#include <filesystem>

#include "chrono/utils/ChUtils.h"
#include "chrono/input_output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/input_output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserSphYAML.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserSphYAML::ChParserSphYAML(const std::string& yaml_filename, bool verbose)
    : ChParserCfdYAML(verbose),
      m_has_wavetank(false),
      m_depth_based_pressure(false),
      m_initial_velocity(false),
      m_velocity(VNULL),
      m_loaded(false),
      m_solver_loaded(false),
      m_model_loaded(false) {
    SetVerbose(verbose);
    LoadFile(yaml_filename);
}

ChParserSphYAML::~ChParserSphYAML() {}

// -----------------------------------------------------------------------------

void ChParserSphYAML::LoadFile(const std::string& yaml_filename) {
    YAML::Node yaml;

    // Load SPH YAML file
    yaml = YAML::LoadFile(yaml_filename);
    m_file_handler.SetReferenceDirectory(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check the YAML file if of type "SPH"
    ChAssertAlways(yaml["type"]);
    auto type = ReadYamlFileType(yaml["type"]);
    ChAssertAlways(type == ChParserYAML::YamlFileType::SPH);

    // Load simulation, output, and run-time visualization data
    LoadSimData(yaml);

    // Load SPH model YAML file
    {
        ChAssertAlways(yaml["model"]);
        auto model_fname = yaml["model"].as<std::string>();
        auto model_filename = m_file_handler.GetReferenceDirectory() + "/" + model_fname;
        auto path = std::filesystem::path(model_filename);
        if (!exists(path) || !is_regular_file(path)) {
            cerr << "Error: file '" << model_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\n[ChParserSphYAML] Loading Chrono SPH model from: '" << yaml_filename << "'\n" << endl;
        }
        auto model = YAML::LoadFile(model_filename);
        ChAssertAlways(model["chrono-version"]);
        CheckVersion(model["chrono-version"]);
        LoadModelData(model);
    }

    // Load solver YAML file
    {
        ChAssertAlways(yaml["solver"]);
        auto solver_fname = yaml["solver"].as<std::string>();
        auto solver_filename = m_file_handler.GetReferenceDirectory() + "/" + solver_fname;
        auto path = std::filesystem::path(solver_filename);
        if (!exists(path) || !is_regular_file(path)) {
            cerr << "Error: file '" << solver_filename << "' not found." << endl;
            throw std::runtime_error("File not found");
        }
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\n[ChParserSphYAML] Loading Chrono SPH solver from: " << solver_filename << "\n" << endl;
        }
        auto solver = YAML::LoadFile(solver_filename);
        ChAssertAlways(solver["chrono-version"]);
        CheckVersion(solver["chrono-version"]);
        LoadSolverData(solver);
    }

    if (m_verbose) {
        m_sim.PrintInfo();
        cout << endl;
        m_vis_settings.PrintInfo();
#ifdef CHRONO_VSG
        m_visSPH_settings.PrintInfo();
#endif
        cout << endl;
        m_output_settings.PrintInfo();
    }

    m_loaded = true;
}

void ChParserSphYAML::LoadSimData(const YAML::Node& yaml) {
    // Read common simulation settings
    ChParserYAML::LoadSimData(yaml);

    // Simulation settings (required)
    if (yaml["simulation"]) {
        auto sim = yaml["simulation"];
        if (sim["end_time"])
            m_sim.end_time = sim["end_time"].as<double>();
        if (sim["gravity"])
            m_sim.gravity = ReadVector(sim["gravity"]);
    }

    // SPH-specific run-time visualization (optional)
    if (yaml["visualization"]) {
#ifdef CHRONO_VSG
        m_visSPH_settings = fsi::sph::ChSphVisualizationVSG::Settings::Read(yaml["visualization"]);
#else
        m_vis_settings.render = false;
#endif
    }
}

void ChParserSphYAML::LoadSolverData(const YAML::Node& yaml) {
    // Base SPH parameters
    if (yaml["sph"]) {
        auto a = yaml["sph"];
        if (a["eos_type"])
            m_sim.sph.eos_type = ReadEosType(a["eos_type"]);
        if (a["use_delta_sph"])
            m_sim.sph.use_delta_sph = a["use_delta_sph"].as<bool>();
        if (a["delta_sph_coefficient"])
            m_sim.sph.delta_sph_coefficient = a["delta_sph_coefficient"].as<double>();
        if (a["max_velocity"])
            m_sim.sph.max_velocity = a["max_velocity"].as<double>();
        if (a["min_distance_coefficient"])
            m_sim.sph.min_distance_coefficient = a["min_distance_coefficient"].as<double>();
        if (a["density_reinit_steps"])
            m_sim.sph.density_reinit_steps = a["density_reinit_steps"].as<int>();
        if (a["use_density_based_projection"])
            m_sim.sph.use_density_based_projection = a["use_density_based_projection"].as<bool>();
        if (a["free_surface_threshold"])
            m_sim.sph.free_surface_threshold = a["free_surface_threshold"].as<double>();
    }

    // SPH kernel parameters
    if (yaml["kernel"]) {
        auto a = yaml["kernel"];
        if (a["kernel_type"])
            m_sim.sph.kernel_type = ReadKernelType(a["kernel_type"]);
        if (a["initial_spacing"])
            m_sim.sph.initial_spacing = a["initial_spacing"].as<double>();
        if (a["d0_multiplier"])
            m_sim.sph.d0_multiplier = a["d0_multiplier"].as<double>();
    }

    // SPH discretization parameters
    if (yaml["discretization"]) {
        auto a = yaml["discretization"];
        if (a["use_consistent_gradient_discretization"])
            m_sim.sph.use_consistent_gradient_discretization = a["use_consistent_gradient_discretization"].as<bool>();
        if (a["use_consistent_laplacian_discretization"])
            m_sim.sph.use_consistent_laplacian_discretization = a["use_consistent_laplacian_discretization"].as<bool>();
    }

    // Boundary condition parameters
    if (yaml["boundary_conditions"]) {
        auto a = yaml["boundary_conditions"];
        if (a["boundary_method"])
            m_sim.sph.boundary_method = ReadBoundaryMethod(a["boundary_method"]);
        if (a["num_bce_layers"])
            m_sim.sph.num_bce_layers = a["num_bce_layers"].as<int>();
    }

    // Integration parameters
    if (yaml["integration"]) {
        auto a = yaml["integration"];
        ChAssertAlways(a["time_step"]);
        m_sim.time_step = a["time_step"].as<double>();
        if (a["integration_scheme"])
            m_sim.sph.integration_scheme = ReadIntegrationScheme(a["integration_scheme"]);
        if (a["use_variable_time_step"])
            m_sim.sph.use_variable_time_step = a["use_variable_time_step"].as<bool>();
    }

    // Proximity search
    if (yaml["proximity_search"]) {
        auto a = yaml["proximity_search"];
        if (a["num_proximity_search_steps"])
            m_sim.sph.num_proximity_search_steps = a["num_proximity_search_steps"].as<int>();
    }

    // Particle shifting
    if (yaml["particle_shifting"]) {
        auto a = yaml["particle_shifting"];
        if (a["shifting_method"])
            m_sim.sph.shifting_method = ReadShiftingMethod(a["shifting_method"]);
        if (a["shifting_xsph_eps"])
            m_sim.sph.shifting_xsph_eps = a["shifting_xsph_eps"].as<double>();
        if (a["shifting_ppst_push"])
            m_sim.sph.shifting_ppst_push = a["shifting_ppst_push"].as<double>();
        if (a["shifting_ppst_pull"])
            m_sim.sph.shifting_ppst_pull = a["shifting_ppst_pull"].as<double>();
        if (a["shifting_beta_implicit"])
            m_sim.sph.shifting_beta_implicit = a["shifting_beta_implicit"].as<double>();
        if (a["shifting_diffusion_A"])
            m_sim.sph.shifting_diffusion_A = a["shifting_diffusion_A"].as<double>();
        if (a["shifting_diffusion_AFSM"])
            m_sim.sph.shifting_diffusion_AFSM = a["shifting_diffusion_AFSM"].as<double>();
        if (a["artificial_viscosity"])
            m_sim.sph.shifting_diffusion_AFST = a["shifting_diffusion_AFST"].as<double>();
    }

    // Viscosity parameters
    if (yaml["viscosity"]) {
        auto a = yaml["viscosity"];
        if (a["viscosity_method"])
            m_sim.sph.viscosity_method = ReadViscosityMethod(a["viscosity_method"]);
        if (a["artificial_viscosity"])
            m_sim.sph.artificial_viscosity = a["artificial_viscosity"].as<double>();
    }

    m_solver_loaded = true;
}

void ChParserSphYAML::LoadModelData(const YAML::Node& yaml) {
    // Check a model object exists
    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    // Physics problem type is required
    ChAssertAlways(model["physics_problem"]);
    m_material.physics_problem = ReadPhysicsProblemType(model["physics_problem"]);

    // Problem geometry is required
    ChAssertAlways(model["geometry_type"]);
    m_geometry_type = ReadGeometryType(model["geometry_type"]);

    if (model["name"])
        m_name = model["name"].as<std::string>();

    if (model["angle_degrees"])
        m_use_degrees = model["angle_degrees"].as<bool>();

    m_file_handler.Read(model);

    if (m_verbose) {
        cout << "model name: '" << m_name << "'" << endl;
        cout << "angles in degrees? " << (m_use_degrees ? "true" : "false") << endl;
        m_file_handler.PrintInfo();
    }

    // Read fluid material properties
    if (model["fluid_properties"]) {
        if (m_verbose)
            cout << "read fluid properties" << endl;

        auto a = model["fluid properties"];
        if (a["density"])
            m_material.fluid_props.density = a["density"].as<double>();
        if (a["viscosity"])
            m_material.fluid_props.viscosity = a["viscosity"].as<double>();
        if (a["char_length"])
            m_material.fluid_props.char_length = a["char_length"].as<double>();
    }

    // Read soil material properties
    if (model["soil_properties"]) {
        if (m_verbose)
            cout << "read soil properties" << endl;

        auto a = model["soil properties"];
        if (a["density"])
            m_material.soil_props.density = a["density"].as<double>();
        if (a["Young_modulus"])
            m_material.soil_props.Young_modulus = a["Young_modulus"].as<double>();
        if (a["Poisson_ratio"])
            m_material.soil_props.Poisson_ratio = a["Poisson_ratio"].as<double>();
        if (a["mu_I0"])
            m_material.soil_props.mu_I0 = a["mu_I0"].as<double>();
        if (a["mu_fric_s"])
            m_material.soil_props.mu_fric_s = a["mu_fric_s"].as<double>();
        if (a["mu_fric_2"])
            m_material.soil_props.mu_fric_2 = a["mu_fric_2"].as<double>();
        if (a["average_diam"])
            m_material.soil_props.average_diam = a["average_diam"].as<double>();
        if (a["cohesion_coefficient"])
            m_material.soil_props.cohesion_coeff = a["cohesion_coefficient"].as<double>();
    }

    // Read SPH state initialization settings
    if (model["initial_states"]) {
        if (m_verbose)
            cout << "read initial state settings" << endl;

        auto a = model["initial_states"];
        if (a["depth_based_pressure"]) {
            m_depth_based_pressure = true;
            ChAssertAlways(a["zero_height"]);
            m_zero_height = a["zero_height"].as<double>();
        }
        if (a["initial_velocity"]) {
            m_initial_velocity = true;
            m_velocity = ReadVector(a["initial_velocity"]);
        }
    }

    // Check first if wavetank
    m_has_wavetank = false;
    if (model["wave_tank"]) {
        if (m_verbose)
            cout << "read wave tank settings" << endl;

        // A wave tank requires a CFD problem in Cartesian geometry
        ChAssertAlways(m_material.physics_problem == fsi::sph::PhysicsProblem::CFD);
        ChAssertAlways(m_geometry_type == GeometryType::CARTESIAN);

        auto a = model["wave_tank"];
        ChAssertAlways(a["type"]);
        ChAssertAlways(a["tank_dimensions"]);
        ChAssertAlways(a["water_depth"]);
        ChAssertAlways(a["actuation_function"]);
        m_wavetank.type = ReadWavetankType(a["type"]);
        m_wavetank.container.dimensions = ReadVector(a["tank_dimensions"]);
        if (a["tank_origin"])
            m_wavetank.container.origin = ReadVector(a["tank_origin"]);
        m_wavetank.depth = a["water_depth"].as<double>();
        if (a["end_wall"])
            m_wavetank.end_wall = a["end_wall"].as<bool>();
        if (a["profile"]) {
            ChAssertAlways(a["profile"].IsSequence());
            auto num_points = a["profile"].size();
            m_wavetank.profile = chrono_types::make_shared<ChFunctionInterp>();
            for (size_t i = 0; i < num_points; i++)
                m_wavetank.profile->AddPoint(a["profile"][i][0].as<double>(), a["profile"][i][1].as<double>());
        }
        m_wavetank.actuation = ReadFunction(a["actuation_function"], m_use_degrees);
        if (a["actuation_function"]["delay"])
            m_wavetank.actuation_delay = a["actuation_function"]["delay"].as<double>();

        m_has_wavetank = true;
    }

    // Read fluid domain, optional container, and optional computational domain settings (unless a wave tank)
    bool has_walls = false;
    if (!m_has_wavetank) {
        // Fluid domain
        ChAssertAlways(model["fluid_domain"]);
        {
            if (m_verbose)
                cout << "read fluid domain settings" << endl;

            auto a = model["fluid_domain"];
            switch (m_geometry_type) {
                case GeometryType::CARTESIAN:
                    ChAssertAlways(a["dimensions"]);
                    m_geometry.fluid_domain_cartesian = chrono_types::make_unique<BoxDomain>();
                    m_geometry.fluid_domain_cartesian->dimensions = ReadVector(a["dimensions"]);
                    if (a["box_origin"]) {
                        m_geometry.fluid_domain_cartesian->origin = ReadVector(a["box_origin"]);
                    } else {
                        m_geometry.fluid_domain_cartesian->origin = VNULL;
                    }
                    if (a["box_walls"]) {
                        m_geometry.fluid_domain_cartesian->wall_code = ReadWallFlagsCartesian(a["box_walls"]);
                        has_walls = (m_geometry.fluid_domain_cartesian->wall_code != fsi::sph::BoxSide::NONE);
                    } else {
                        m_geometry.fluid_domain_cartesian->wall_code = fsi::sph::BoxSide::NONE;
                    }
                    break;
                case GeometryType::CYLINDRICAL:
                    ChAssertAlways(a["inner_radius"]);
                    ChAssertAlways(a["outer_radius"]);
                    ChAssertAlways(a["height"]);
                    m_geometry.fluid_domain_cylindrical = chrono_types::make_unique<AnnulusDomain>();
                    m_geometry.fluid_domain_cylindrical->inner_radius = a["inner_radius"].as<double>();
                    m_geometry.fluid_domain_cylindrical->outer_radius = a["outer_radius"].as<double>();
                    m_geometry.fluid_domain_cylindrical->height = a["height"].as<double>();
                    if (a["cyl_origin"]) {
                        m_geometry.fluid_domain_cylindrical->origin = ReadVector(a["cyl_origin"]);
                    } else {
                        m_geometry.fluid_domain_cylindrical->origin = VNULL;
                    }
                    if (a["cyl_walls"]) {
                        m_geometry.fluid_domain_cylindrical->wall_code = ReadWallFlagsCylindrical(a["cyl_walls"]);
                        has_walls = (m_geometry.fluid_domain_cylindrical->wall_code != fsi::sph::CylSide::NONE);
                    } else {
                        m_geometry.fluid_domain_cylindrical->wall_code = fsi::sph::CylSide::NONE;
                    }
                    break;
            }
        }

        // Optional container
        if (model["container"]) {
            if (m_verbose)
                cout << "read container settings" << endl;

            // Note: A container definition is not allowed if wall boundaries have already been defined!
            ChAssertAlways(!has_walls);

            auto a = model["container"];
            switch (m_geometry_type) {
                case GeometryType::CARTESIAN:
                    m_geometry.container_cartesian = chrono_types::make_unique<BoxDomain>();
                    ChAssertAlways(a["dimensions"]);
                    m_geometry.container_cartesian->dimensions = ReadVector(a["dimensions"]);
                    if (a["box_origin"]) {
                        m_geometry.container_cartesian->origin = ReadVector(a["box_origin"]);
                    } else {
                        m_geometry.container_cartesian->origin = VNULL;
                    }
                    if (a["box_walls"]) {
                        m_geometry.container_cartesian->wall_code = ReadWallFlagsCartesian(a["box_walls"]);
                        has_walls = (m_geometry.container_cartesian->wall_code != fsi::sph::BoxSide::NONE);
                    } else {
                        m_geometry.container_cartesian->wall_code = fsi::sph::BoxSide::NONE;
                    }
                    break;
                case GeometryType::CYLINDRICAL:
                    m_geometry.fluid_domain_cylindrical = chrono_types::make_unique<AnnulusDomain>();
                    ChAssertAlways(a["inner_radius"]);
                    ChAssertAlways(a["outer_radius"]);
                    ChAssertAlways(a["height"]);
                    m_geometry.fluid_domain_cylindrical->inner_radius = a["inner_radius"].as<double>();
                    m_geometry.fluid_domain_cylindrical->outer_radius = a["outer_radius"].as<double>();
                    m_geometry.fluid_domain_cylindrical->height = a["height"].as<double>();
                    if (a["cyl_origin"]) {
                        m_geometry.fluid_domain_cylindrical->origin = ReadVector(a["cyl_origin"]);
                    } else {
                        m_geometry.fluid_domain_cylindrical->origin = VNULL;
                    }
                    if (a["cyl_walls"]) {
                        m_geometry.fluid_domain_cylindrical->wall_code = ReadWallFlagsCylindrical(a["cyl_walls"]);
                        has_walls = (m_geometry.fluid_domain_cylindrical->wall_code != fsi::sph::CylSide::NONE);
                    } else {
                        m_geometry.fluid_domain_cylindrical->wall_code = fsi::sph::CylSide::NONE;
                    }
                    break;
            }
        }

        // Read optional computational domain settings
        if (model["computational_domain"]) {
            if (m_verbose)
                cout << "read computational domain settings" << endl;

            auto a = model["computational_domain"];
            m_geometry.computational_domain = chrono_types::make_unique<ComputationalDomain>();
            ChAssertAlways(a["aabb_min"]);
            ChAssertAlways(a["aabb_max"]);
            m_geometry.computational_domain->aabb.min = ReadVector(a["aabb_min"]);
            m_geometry.computational_domain->aabb.max = ReadVector(a["aabb_max"]);
            if (a["x_bc_type"]) {
                auto x_bc_type = ReadBoundaryConditionType(a["x_bc_type"]);
                if (x_bc_type != fsi::sph::BCType::NONE && m_geometry_type == GeometryType::CARTESIAN) {
                    ChAssertAlways((m_geometry.fluid_domain_cartesian->wall_code & static_cast<int>(fsi::sph::BoxSide::X_NEG)) == 0);
                    ChAssertAlways((m_geometry.fluid_domain_cartesian->wall_code & static_cast<int>(fsi::sph::BoxSide::X_POS)) == 0);
                }
                m_geometry.computational_domain->bc_type.x = x_bc_type;
            }
            if (a["y_bc_type"]) {
                auto y_bc_type = ReadBoundaryConditionType(a["y_bc_type"]);
                if (y_bc_type != fsi::sph::BCType::NONE && m_geometry_type == GeometryType::CARTESIAN) {
                    ChAssertAlways((m_geometry.fluid_domain_cartesian->wall_code & static_cast<int>(fsi::sph::BoxSide::Y_NEG)) == 0);
                    ChAssertAlways((m_geometry.fluid_domain_cartesian->wall_code & static_cast<int>(fsi::sph::BoxSide::Y_POS)) == 0);
                }
                m_geometry.computational_domain->bc_type.y = y_bc_type;
            }
            if (a["z_bc_type"]) {
                auto z_bc_type = ReadBoundaryConditionType(a["z_bc_type"]);
                if (z_bc_type != fsi::sph::BCType::NONE && m_geometry_type == GeometryType::CARTESIAN) {
                    ChAssertAlways((m_geometry.fluid_domain_cartesian->wall_code & static_cast<int>(fsi::sph::BoxSide::Z_NEG)) == 0);
                    ChAssertAlways((m_geometry.fluid_domain_cartesian->wall_code & static_cast<int>(fsi::sph::BoxSide::Z_POS)) == 0);
                }
                m_geometry.computational_domain->bc_type.z = z_bc_type;
            }
        }
    }

    if (m_verbose) {
        cout << endl;
        m_material.PrintInfo();
        cout << endl;
        if (m_has_wavetank)
            m_wavetank.PrintInfo();
        else
            m_geometry.PrintInfo();
    }

    m_model_loaded = true;
}

// -----------------------------------------------------------------------------

// Wrapper function for wave maker actuation with delay.
class WavemakerFunction : public ChFunction {
  public:
    WavemakerFunction() : delay(0), actuation(nullptr) {}
    WavemakerFunction(double delay, std::shared_ptr<ChFunction> actuation) : delay(delay), actuation(actuation) {}

    virtual WavemakerFunction* Clone() const override { return new WavemakerFunction(); }

    virtual double GetVal(double t) const override {
        double val = 0;
        if (t >= delay)
            val = actuation->GetVal(t - delay);
        return val;
    }

  private:
    double delay;
    std::shared_ptr<ChFunction> actuation;
};

// Callback for setting initial SPH particle properties
class SPHPropertiesCallback : public fsi::sph::ChFsiProblemSPH::ParticlePropertiesCallback {
  public:
    SPHPropertiesCallback(bool set_pressure, double zero_height, bool set_velocity, const ChVector3d& init_velocity)
        : ParticlePropertiesCallback(), set_pressure(set_pressure), zero_height(zero_height), set_velocity(set_velocity), init_velocity(init_velocity) {}

    virtual void set(const fsi::sph::ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override {
        if (set_pressure) {
            double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
            double c2 = sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed();
            p0 = sysSPH.GetDensity() * gz * (zero_height - pos.z());
            rho0 = sysSPH.GetDensity() + p0 / c2;
            mu0 = sysSPH.GetViscosity();
        }

        if (set_velocity) {
            v0 = init_velocity;
        }
    }

    bool set_pressure;
    double zero_height;
    bool set_velocity;
    ChVector3d init_velocity;
};

// Callback for wave tank profile definition.
class WaveTankProfile : public fsi::sph::ChFsiProblemWavetank::Profile {
  public:
    WaveTankProfile(const ChFunctionInterp& fun) : fun(fun) {}
    virtual double operator()(double x) { return fun.GetVal(x); }

  private:
    const ChFunctionInterp& fun;
};

std::shared_ptr<fsi::sph::ChFsiProblemSPH> ChParserSphYAML::CreateFsiProblemSPH(bool initialize) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserSphYAML] Create ChFSIProblemSPH\n" << endl;
    }

    if (!m_model_loaded) {
        cerr << "[ChParserSphYAML::CreateFsiProblemSPH] Error: no YAML model file loaded." << endl;
        throw std::runtime_error("No YAML model file loaded");
    }

    if (!m_solver_loaded) {
        cerr << "[ChParserSphYAML::CreateFsiProblemSPH] Error: no YAML solver file loaded." << endl;
        throw std::runtime_error("No YAML solver file loaded");
    }

    // Create a Chrono FSI SPH problem of specified type with no MBS attached
    if (m_has_wavetank) {
        m_fsi_problem = chrono_types::make_shared<fsi::sph::ChFsiProblemWavetank>(m_sim.sph.initial_spacing);
    } else {
        switch (m_geometry_type) {
            case GeometryType::CARTESIAN:
                m_fsi_problem = chrono_types::make_shared<fsi::sph::ChFsiProblemCartesian>(m_sim.sph.initial_spacing);
                break;
            case GeometryType::CYLINDRICAL:
                m_fsi_problem = chrono_types::make_shared<fsi::sph::ChFsiProblemCylindrical>(m_sim.sph.initial_spacing);
                break;
        }
    }

    m_fsi_problem->SetVerbose(m_verbose);

    // Set simulation parameters
    m_fsi_problem->SetGravitationalAcceleration(m_sim.gravity);
    m_fsi_problem->SetStepSizeCFD(m_sim.time_step);
    m_fsi_problem->SetSPHParameters(m_sim.sph);

    // Set material properties
    switch (m_material.physics_problem) {
        case fsi::sph::PhysicsProblem::CFD:
            m_fsi_problem->SetCfdSPH(m_material.fluid_props);
            break;
        case fsi::sph::PhysicsProblem::CRM:
            m_fsi_problem->SetElasticSPH(m_material.soil_props);
            break;
    }

    // Set callback for initial states
    if (m_depth_based_pressure || m_initial_velocity) {
        m_fsi_problem->RegisterParticlePropertiesCallback(chrono_types::make_shared<SPHPropertiesCallback>(m_depth_based_pressure, m_zero_height, m_initial_velocity, m_velocity));
    }

    if (m_has_wavetank) {
        // Construct the wavetank
        auto fsi_problem = std::static_pointer_cast<fsi::sph::ChFsiProblemWavetank>(m_fsi_problem);
        if (m_wavetank.profile)
            fsi_problem->SetProfile(chrono_types::make_shared<WaveTankProfile>(*m_wavetank.profile), m_wavetank.end_wall);
        auto actuation = chrono_types::make_shared<WavemakerFunction>(m_wavetank.actuation_delay, m_wavetank.actuation);
        fsi_problem->ConstructWaveTank(m_wavetank.type, m_wavetank.container.origin, m_wavetank.container.dimensions, m_wavetank.depth, actuation);
    } else {
        // Construct the fluid domain, optional container, and optional computational domain
        switch (m_geometry_type) {
            case GeometryType::CARTESIAN: {
                auto fsi_problem = std::static_pointer_cast<fsi::sph::ChFsiProblemCartesian>(m_fsi_problem);
                fsi_problem->Construct(m_geometry.fluid_domain_cartesian->dimensions,  //
                                       m_geometry.fluid_domain_cartesian->origin,      //
                                       m_geometry.fluid_domain_cartesian->wall_code);
                if (m_geometry.container_cartesian) {
                    fsi_problem->AddBoxContainer(m_geometry.container_cartesian->dimensions,  //
                                                 m_geometry.container_cartesian->origin,      //
                                                 m_geometry.container_cartesian->wall_code);
                }
                break;
            }
            case GeometryType::CYLINDRICAL: {
                auto fsi_problem = std::static_pointer_cast<fsi::sph::ChFsiProblemCylindrical>(m_fsi_problem);
                fsi_problem->Construct(m_geometry.fluid_domain_cylindrical->inner_radius,  //
                                       m_geometry.fluid_domain_cylindrical->outer_radius,  //
                                       m_geometry.fluid_domain_cylindrical->height,        //
                                       m_geometry.fluid_domain_cylindrical->origin,        //
                                       m_geometry.fluid_domain_cylindrical->wall_code);
                if (m_geometry.container_cylindrical) {
                    fsi_problem->AddCylindricalContainer(m_geometry.container_cylindrical->inner_radius,  //
                                                         m_geometry.container_cylindrical->outer_radius,  //
                                                         m_geometry.container_cylindrical->height,        //
                                                         m_geometry.container_cylindrical->origin,        //
                                                         m_geometry.container_cylindrical->wall_code);
                }
                break;
            }
        }

        // Explicitly set computational domain (if provided)
        if (m_geometry.computational_domain) {
            m_fsi_problem->SetComputationalDomain(m_geometry.computational_domain->aabb, m_geometry.computational_domain->bc_type);
        }
    }

    // Initialize FSI problem
    if (initialize)
        m_fsi_problem->Initialize();

    return m_fsi_problem;
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG

const fsi::sph::ChSphVisualizationVSG::Settings& ChParserSphYAML::GetSphVisualizationSettings() const {
    return m_visSPH_settings;
}

const fsi::sph::ChFsiFluidSystemSPH::SplashsurfParameters& ChParserSphYAML::GetSplashsurfParameters() const {
    return m_visSPH_settings.splashsurf_params;
}

std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> ChParserSphYAML::GetVisualizationPlugin() const {
    auto vis = chrono_types::make_shared<fsi::sph::ChSphVisualizationVSG>(m_fsi_problem->GetFsiSystemSPH().get());

    vis->EnableFluidMarkers(m_visSPH_settings.sph_markers);
    vis->EnableBoundaryMarkers(m_visSPH_settings.bndry_bce_markers);
    vis->EnableRigidBodyMarkers(m_visSPH_settings.rigid_bce_markers);

    if (m_visSPH_settings.color_callback)
        vis->SetSPHColorCallback(m_visSPH_settings.color_callback, m_visSPH_settings.colormap);
    if (m_visSPH_settings.visibility_callback_sph)
        vis->SetSPHVisibilityCallback(m_visSPH_settings.visibility_callback_sph);
    if (m_visSPH_settings.visibility_callback_bce)
        vis->SetBCEVisibilityCallback(m_visSPH_settings.visibility_callback_bce);
    return vis;
}

#endif

// -----------------------------------------------------------------------------

void ChParserSphYAML::WriteOutput(int frame, double time) {
    ChParserYAML::WriteOutput(frame, time);

    //// TODO
}

// -----------------------------------------------------------------------------

ChParserSphYAML::MaterialProperties::MaterialProperties() {}

void ChParserSphYAML::MaterialProperties::PrintInfo() const {
    switch (physics_problem) {
        case fsi::sph::PhysicsProblem::CFD:
            cout << "Fluid parameters" << endl;
            cout << "  properties" << endl;
            cout << "     density:               " << fluid_props.density << endl;
            cout << "     viscosity:             " << fluid_props.viscosity << endl;
            cout << "     characteristic length: " << fluid_props.char_length << endl;
            break;
        case fsi::sph::PhysicsProblem::CRM:
            cout << "Soil parameters" << endl;
            cout << "  properties" << endl;
            cout << "      density:               " << soil_props.density << endl;
            cout << "      Young modulus:         " << soil_props.Young_modulus << endl;
            cout << "      Poisson ratio:         " << soil_props.Poisson_ratio << endl;
            cout << "      mu I0 :                " << soil_props.mu_I0 << endl;
            cout << "      mu fric_s :            " << soil_props.mu_fric_s << endl;
            cout << "      mu fric_2 :            " << soil_props.mu_fric_2 << endl;
            cout << "      average diameter :     " << soil_props.average_diam << endl;
            cout << "      cohesion coefficient : " << soil_props.cohesion_coeff << endl;
            break;
    }
}

ChParserSphYAML::ProblemGeometry::ProblemGeometry() {}

void ChParserSphYAML::ProblemGeometry::PrintInfo() const {
    cout << "Problem geometry" << endl;
    if (fluid_domain_cartesian) {
        cout << "  domain (CARTESIAN)" << endl;
        cout << "      dimensions: " << fluid_domain_cartesian->dimensions << endl;
        cout << "      origin:     " << fluid_domain_cartesian->origin << endl;
        cout << "      wall code:  " << fluid_domain_cartesian->wall_code << endl;
    } else if (fluid_domain_cylindrical) {
        cout << "  domain (CYLINDRICAL)" << endl;
        cout << "      inner radius: " << fluid_domain_cylindrical->inner_radius << endl;
        cout << "      outer radius: " << fluid_domain_cylindrical->outer_radius << endl;
        cout << "      height:       " << fluid_domain_cylindrical->height << endl;
        cout << "      origin:       " << fluid_domain_cylindrical->origin << endl;
        cout << "      wall code:    " << fluid_domain_cylindrical->wall_code << endl;
    }

    if (container_cartesian) {
        cout << "  container (CARTESIAN)" << endl;
        cout << "      dimensions: " << container_cartesian->dimensions << endl;
        cout << "      origin:     " << container_cartesian->origin << endl;
        cout << "      wall code:  " << container_cartesian->wall_code << endl;
    } else if (container_cylindrical) {
        cout << "  container (CYLINDRICAL)" << endl;
        cout << "      inner radius: " << container_cylindrical->inner_radius << endl;
        cout << "      outer radius: " << container_cylindrical->outer_radius << endl;
        cout << "      height:       " << container_cylindrical->height << endl;
        cout << "      origin:       " << container_cylindrical->origin << endl;
        cout << "      wall code:    " << container_cylindrical->wall_code << endl;
    }
}

ChParserSphYAML::Wavetank::Wavetank() : end_wall(true), actuation_delay(0) {}

void ChParserSphYAML::Wavetank::PrintInfo() const {
    cout << "Wavetank settings" << endl;
    if (type == fsi::sph::ChFsiProblemWavetank::WavemakerType::PISTON)
        cout << "  type:             PISTON " << endl;
    else
        cout << "  type:             FLAP " << endl;
    cout << "  dimensions:      " << container.dimensions << endl;
    cout << "  origin:          " << container.origin << endl;
    cout << "  depth:           " << depth << endl;
    cout << "  end wall:        " << end_wall << endl;
    cout << "  actuation delay: " << actuation_delay << endl;
}

ChParserSphYAML::SimParams::SimParams() : gravity({0, 0, -9.8}), time_step(1e-4), end_time(-1) {}

void ChParserSphYAML::SimParams::PrintInfo() const {
    cout << "simulation end time:        " << (end_time < 0 ? "infinite" : std::to_string(end_time)) << endl;
    cout << endl;

    cout << "SPH settings" << endl;
    cout << "  integration time step:      " << time_step << endl;

    //// TODO
}

// =============================================================================

ChParserSphYAML::GeometryType ChParserSphYAML::ReadGeometryType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "CARTESIAN")
        return GeometryType::CARTESIAN;
    if (val == "CYLINDRICAL")
        return GeometryType::CYLINDRICAL;
    return GeometryType::CARTESIAN;
}

fsi::sph::PhysicsProblem ChParserSphYAML::ReadPhysicsProblemType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "CFD")
        return fsi::sph::PhysicsProblem::CFD;
    if (val == "CRM")
        return fsi::sph::PhysicsProblem::CRM;
    return fsi::sph::PhysicsProblem::CFD;
}

fsi::sph::ChFsiProblemWavetank::WavemakerType ChParserSphYAML::ReadWavetankType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "PISTON")
        return fsi::sph::ChFsiProblemWavetank::WavemakerType::PISTON;
    if (val == "FLAP")
        return fsi::sph::ChFsiProblemWavetank::WavemakerType::FLAP;
    return fsi::sph::ChFsiProblemWavetank::WavemakerType::PISTON;
}

fsi::sph::EosType ChParserSphYAML::ReadEosType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "ISOTHERMAL")
        return fsi::sph::EosType::ISOTHERMAL;
    if (val == "TAIT")
        return fsi::sph::EosType::TAIT;
    return fsi::sph::EosType::ISOTHERMAL;
}

fsi::sph::KernelType ChParserSphYAML::ReadKernelType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "QUADRATIC")
        return fsi::sph::KernelType::QUADRATIC;
    if (val == "CUBIC_SPLINE")
        return fsi::sph::KernelType::CUBIC_SPLINE;
    if (val == "QUINTIC_SPLINE")
        return fsi::sph::KernelType::QUINTIC_SPLINE;
    if (val == "WENDLAND")
        return fsi::sph::KernelType::WENDLAND;
    return fsi::sph::KernelType::CUBIC_SPLINE;
}

fsi::sph::IntegrationScheme ChParserSphYAML::ReadIntegrationScheme(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "EULER")
        return fsi::sph::IntegrationScheme::EULER;
    if (val == "RK2")
        return fsi::sph::IntegrationScheme::RK2;
    if (val == "VERLET")
        return fsi::sph::IntegrationScheme::VERLET;
    if (val == "SYMPLECTIC")
        return fsi::sph::IntegrationScheme::SYMPLECTIC;
    if (val == "IMPLICIT_SPH")
        return fsi::sph::IntegrationScheme::IMPLICIT_SPH;
    return fsi::sph::IntegrationScheme::RK2;
}

fsi::sph::BoundaryMethod ChParserSphYAML::ReadBoundaryMethod(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "ADAMI")
        return fsi::sph::BoundaryMethod::ADAMI;
    if (val == "HOLMES")
        return fsi::sph::BoundaryMethod::HOLMES;
    return fsi::sph::BoundaryMethod::ADAMI;
}

fsi::sph::ShiftingMethod ChParserSphYAML::ReadShiftingMethod(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "NONE")
        return fsi::sph::ShiftingMethod::NONE;
    if (val == "PPST")
        return fsi::sph::ShiftingMethod::PPST;
    if (val == "XSPH")
        return fsi::sph::ShiftingMethod::XSPH;
    if (val == "PPST_XSPH")
        return fsi::sph::ShiftingMethod::PPST_XSPH;
    if (val == "DIFFUSION")
        return fsi::sph::ShiftingMethod::DIFFUSION;
    if (val == "NONE")
        return fsi::sph::ShiftingMethod::DIFFUSION_XSPH;
    return fsi::sph::ShiftingMethod::XSPH;
}

fsi::sph::ViscosityMethod ChParserSphYAML::ReadViscosityMethod(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "LAMINAR")
        return fsi::sph::ViscosityMethod::LAMINAR;
    if (val == "ARTIFICIAL_UNILATERAL")
        return fsi::sph::ViscosityMethod::ARTIFICIAL_UNILATERAL;
    if (val == "ARTIFICIAL_BILATERAL")
        return fsi::sph::ViscosityMethod::ARTIFICIAL_BILATERAL;
    return fsi::sph::ViscosityMethod::ARTIFICIAL_UNILATERAL;
}

int ChParserSphYAML::ReadWallFlagsCartesian(const YAML::Node& a) {
    int code = fsi::sph::BoxSide::NONE;

    ChAssertAlways(a["x"]);
    ChAssertAlways(a["x"].IsSequence());
    ChAssertAlways(a["x"].size() == 2);
    if (a["x"][0].as<bool>())
        code |= fsi::sph::BoxSide::X_NEG;
    if (a["x"][1].as<bool>())
        code |= fsi::sph::BoxSide::X_POS;

    ChAssertAlways(a["y"]);
    ChAssertAlways(a["y"].IsSequence());
    ChAssertAlways(a["y"].size() == 2);
    if (a["y"][0].as<bool>())
        code |= fsi::sph::BoxSide::Y_NEG;
    if (a["y"][1].as<bool>())
        code |= fsi::sph::BoxSide::Y_POS;

    ChAssertAlways(a["z"]);
    ChAssertAlways(a["z"].IsSequence());
    ChAssertAlways(a["z"].size() == 2);
    if (a["z"][0].as<bool>())
        code |= fsi::sph::BoxSide::Z_NEG;
    if (a["z"][1].as<bool>())
        code |= fsi::sph::BoxSide::Z_POS;

    return code;
}

int ChParserSphYAML::ReadWallFlagsCylindrical(const YAML::Node& a) {
    int code = fsi::sph::CylSide::NONE;

    ChAssertAlways(a["side"]);
    ChAssertAlways(a["side"].IsSequence());
    ChAssertAlways(a["side"].size() == 2);
    if (a["side"][0].as<bool>())
        code |= fsi::sph::CylSide::SIDE_INT;
    if (a["side"][1].as<bool>())
        code |= fsi::sph::CylSide::SIDE_EXT;

    ChAssertAlways(a["z"]);
    ChAssertAlways(a["z"].IsSequence());
    ChAssertAlways(a["z"].size() == 2);
    if (a["z"][0].as<bool>())
        code |= fsi::sph::BoxSide::Z_NEG;
    if (a["z"][1].as<bool>())
        code |= fsi::sph::BoxSide::Z_POS;

    return code;
}

fsi::sph::BCType ChParserSphYAML::ReadBoundaryConditionType(const YAML::Node& a) {
    auto val = ChToUpper(a.as<std::string>());
    if (val == "NONE")
        return fsi::sph::BCType::NONE;
    if (val == "PERIODIC")
        return fsi::sph::BCType::PERIODIC;
    if (val == "INLET_OUTLET")
        return fsi::sph::BCType::INLET_OUTLET;
    return fsi::sph::BCType::NONE;
}

}  // namespace parsers
}  // namespace chrono
