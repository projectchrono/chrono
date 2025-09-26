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

#include "chrono/ChConfig.h"
#include "chrono/ChVersion.h"
#include "chrono/utils/ChUtils.h"
#include "chrono/output/ChOutputASCII.h"
#ifdef CHRONO_HAS_HDF5
    #include "chrono/output/ChOutputHDF5.h"
#endif

#include "chrono_parsers/yaml/ChParserSphYAML.h"
#include "chrono_parsers/yaml/ChParserMbsYAML.h"

#include "chrono_thirdparty/filesystem/path.h"

using std::cout;
using std::cerr;
using std::endl;

namespace chrono {
namespace parsers {

ChParserSphYAML::ChParserSphYAML(const std::string& yaml_model_filename,
                                 const std::string& yaml_sim_filename,
                                 bool verbose)
    : ChParserCfdYAML(verbose),
      m_name("YAML SPH model"),
      m_data_path(DataPathType::ABS),
      m_rel_path("."),
      m_depth_based_pressure(false),
      m_initial_velocity(false),
      m_velocity(VNULL),
      m_sim_loaded(false),
      m_model_loaded(false),
      m_output_dir("") {
    LoadModelFile(yaml_model_filename);
    LoadSimulationFile(yaml_sim_filename);
}

ChParserSphYAML::~ChParserSphYAML() {}

// -----------------------------------------------------------------------------

void ChParserSphYAML::LoadSimulationFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check that the file is an SPH specification
    ChAssertAlways(yaml["fluid_dynamics_solver"]);
    if (ToUpper(yaml["fluid_dynamics_solver"].as<std::string>()) != "SPH") {
        cerr << "Error: file '" << yaml_filename << "' is not an SPH specification file." << endl;
        throw std::runtime_error("Not an SPH specification file");
    }

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check a simulation object exists
    ChAssertAlways(yaml["simulation"]);
    auto sim = yaml["simulation"];

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserSphYAML] Loading Chrono::SPH simulation specification from: " << yaml_filename << "\n"
             << endl;
    }

    ChAssertAlways(sim["time_step"]);
    m_sim.time_step = sim["time_step"].as<double>();
    if (sim["end_time"])
        m_sim.end_time = sim["end_time"].as<double>();
    if (sim["gravity"])
        m_sim.gravity = ChParserMbsYAML::ReadVector(sim["gravity"]);

    // Base SPH parameters
    if (sim["sph"]) {
        auto a = sim["sph"];
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
    if (sim["kernel"]) {
        auto a = sim["kernel"];
        if (a["kernel_type"])
            m_sim.sph.kernel_type = ReadKernelType(a["kernel_type"]);
        if (a["initial_spacing"])
            m_sim.sph.initial_spacing = a["initial_spacing"].as<double>();
        if (a["d0_multiplier"])
            m_sim.sph.d0_multiplier = a["d0_multiplier"].as<double>();
    }

    // SPH discretization parameters
    if (sim["discretization"]) {
        auto a = sim["discretization"];
        if (a["use_consistent_gradient_discretization"])
            m_sim.sph.use_consistent_gradient_discretization = a["use_consistent_gradient_discretization"].as<bool>();
        if (a["use_consistent_laplacian_discretization"])
            m_sim.sph.use_consistent_laplacian_discretization = a["use_consistent_laplacian_discretization"].as<bool>();
    }

    // Boundary condition parameters
    if (sim["boundary_conditions"]) {
        auto a = sim["boundary_conditions"];
        if (a["boundary_method"])
            m_sim.sph.boundary_method = ReadBoundaryMethod(a["boundary_method"]);
        if (a["num_bce_layers"])
            m_sim.sph.num_bce_layers = a["num_bce_layers"].as<int>();
    }

    // Integration parameters
    if (sim["integration"]) {
        auto a = sim["integration"];
        if (a["integration_scheme"])
            m_sim.sph.integration_scheme = ReadIntegrationScheme(a["integration_scheme"]);
        if (a["use_variable_time_step"])
            m_sim.sph.use_variable_time_step = a["use_variable_time_step"].as<bool>();
    }

    // Proximity search
    if (sim["proximity_search"]) {
        auto a = sim["proximity_search"];
        if (a["num_proximity_search_steps"])
            m_sim.sph.num_proximity_search_steps = a["num_proximity_search_steps"].as<int>();
    }

    // Particle shifting
    if (sim["particle_shifting"]) {
        auto a = sim["particle_shifting"];
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
    if (sim["viscosity"]) {
        auto a = sim["viscosity"];
        if (a["viscosity_method"])
            m_sim.sph.viscosity_method = ReadViscosityMethod(a["viscosity_method"]);
        if (a["artificial_viscosity"])
            m_sim.sph.artificial_viscosity = a["artificial_viscosity"].as<double>();
    }

    // Run-time visualization (optional)
    if (sim["visualization"]) {
#ifdef CHRONO_VSG
        m_sim.visualization.render = true;
        auto a = sim["visualization"];

        if (a["sph_markers"])
            m_sim.visualization.sph_markers = a["sph_markers"].as<bool>();
        if (a["rigid_bce_markers"])
            m_sim.visualization.rigid_bce_markers = a["rigid_bce_markers"].as<bool>();
        if (a["flex_bce_markers"])
            m_sim.visualization.flex_bce_markers = a["flex_bce_markers"].as<bool>();
        if (a["bndry_bce_markers"])
            m_sim.visualization.bndry_bce_markers = a["bndry_bce_markers"].as<bool>();
        if (a["active_boxes"])
            m_sim.visualization.active_boxes = a["active_boxes"].as<bool>();

        if (a["color_map"]) {
            auto b = a["color_map"];
            ChAssertAlways(b["type"]);
            auto type = ReadParticleColoringType(b["type"]);
            if (b["map"])
                m_sim.visualization.colormap = ReadColorMapType(b["map"]);
            switch (type) {
                case ParticleColoringType::NONE:
                    break;
                case ParticleColoringType::HEIGHT: {
                    ChAssertAlways(b["min"]);
                    ChAssertAlways(b["max"]);
                    double min = b["min"].as<double>();
                    double max = b["max"].as<double>();
                    ChVector3d up = VECT_Z;
                    if (b["up"])
                        up = ChParserMbsYAML::ReadVector(b["up"]);
                    m_sim.visualization.color_callback =
                        chrono_types::make_shared<fsi::sph::ParticleHeightColorCallback>(min, max, up);
                    break;
                }
                case ParticleColoringType::VELOCITY: {
                    ChAssertAlways(b["min"]);
                    ChAssertAlways(b["max"]);
                    double min = b["min"].as<double>();
                    double max = b["max"].as<double>();
                    m_sim.visualization.color_callback =
                        chrono_types::make_shared<fsi::sph::ParticleVelocityColorCallback>(min, max);
                    break;
                }
                case ParticleColoringType::DENSITY: {
                    ChAssertAlways(b["min"]);
                    ChAssertAlways(b["max"]);
                    double min = b["min"].as<double>();
                    double max = b["max"].as<double>();
                    m_sim.visualization.color_callback =
                        chrono_types::make_shared<fsi::sph::ParticleDensityColorCallback>(min, max);
                    break;
                }
                case ParticleColoringType::PRESSURE: {
                    ChAssertAlways(b["min"]);
                    ChAssertAlways(b["max"]);
                    ChAssertAlways(b["bimodal"]);
                    double min = b["min"].as<double>();
                    double max = b["max"].as<double>();
                    bool bimodal = b["bimodal"].as<bool>();
                    m_sim.visualization.color_callback =
                        chrono_types::make_shared<fsi::sph::ParticlePressureColorCallback>(min, max, bimodal);
                    break;
                }
            }
        }

        if (a["visibility"]) {
            auto b = a["visibility"];

            ChAssertAlways(b["planes"]);
            auto c = b["planes"];
            ChAssertAlways(c.IsSequence());
            std::vector<fsi::sph::MarkerPlanesVisibilityCallback::Plane> planes;
            for (int i = 0; i < c.size(); i++) {
                ChAssertAlways(c[i]["point"]);
                ChAssertAlways(c[i]["normal"]);
                auto point = ChParserMbsYAML::ReadVector(c[i]["point"]);
                auto normal = ChParserMbsYAML::ReadVector(c[i]["normal"]);
                planes.push_back({point, normal});
            }

            bool sph_visibility = true;
            if (b["SPH"])
                sph_visibility = b["SPH"].as<bool>();

            bool bce_visibility = true;
            if (b["BCE"])
                bce_visibility = b["BCE"].as<bool>();

            fsi::sph::MarkerPlanesVisibilityCallback::Mode mode;
            if (b["mode"])
                mode = ReadVisibilityMode(b["mode"]);
            else
                mode = fsi::sph::MarkerPlanesVisibilityCallback::Mode::ALL;

            if (sph_visibility)
                m_sim.visualization.visibility_callback_sph =
                    chrono_types::make_shared<fsi::sph::MarkerPlanesVisibilityCallback>(planes, mode);
            if (bce_visibility)
                m_sim.visualization.visibility_callback_bce =
                    chrono_types::make_shared<fsi::sph::MarkerPlanesVisibilityCallback>(planes, mode);
        }

        if (a["splashsurf"]) {
            m_sim.visualization.use_splashsurf = true;
            m_sim.visualization.splashsurf_params =
                chrono_types::make_unique<fsi::sph::ChFsiFluidSystemSPH::SplashsurfParameters>();
            auto b = a["splashsurf"];
            if (b["smoothing_length"])
                m_sim.visualization.splashsurf_params->smoothing_length = b["smoothing_length"].as<double>();
            if (b["cube_size"])
                m_sim.visualization.splashsurf_params->cube_size = b["cube_size"].as<double>();
            if (b["surface_threshold"])
                m_sim.visualization.splashsurf_params->surface_threshold = b["surface_threshold"].as<double>();
        }

        if (a["output"]) {
            auto b = a["output"];
            if (b["save_images"])
                m_sim.visualization.write_images = b["save_images"].as<bool>();
            if (b["output_directory"])
                m_sim.visualization.image_dir = b["output_directory"].as<std::string>();
        }
#endif
    }

    // Output (optional)
    if (sim["output"]) {
        ChAssertAlways(sim["output"]["type"]);
        m_sim.output.type = ReadOutputType(sim["output"]["type"]);
        if (sim["output"]["mode"])
            m_sim.output.mode = ReadOutputMode(sim["output"]["mode"]);
        if (sim["output"]["fps"])
            m_sim.output.fps = sim["output"]["fps"].as<double>();
        if (sim["output"]["output_directory"])
            m_sim.output.dir = sim["output"]["output_directory"].as<std::string>();
    }

    if (m_verbose)
        m_sim.PrintInfo();

    m_sim_loaded = true;
}

void ChParserSphYAML::LoadModelFile(const std::string& yaml_filename) {
    auto path = filesystem::path(yaml_filename);
    if (!path.exists() || !path.is_file()) {
        cerr << "Error: file '" << yaml_filename << "' not found." << endl;
        throw std::runtime_error("File not found");
    }

    m_script_directory = path.parent_path().str();

    YAML::Node yaml = YAML::LoadFile(yaml_filename);

    // Check version compatibility
    ChAssertAlways(yaml["chrono-version"]);
    CheckVersion(yaml["chrono-version"]);

    // Check a model object exists
    ChAssertAlways(yaml["model"]);
    auto model = yaml["model"];

    // Physics problem type is required
    ChAssertAlways(model["physics_problem"]);
    m_fluid.physics_problem = ReadPhysicsProblemType(model["physics_problem"]);

    // Problem geometry is required
    ChAssertAlways(model["geometry_type"]);
    m_geometry_type = ReadGeometryType(model["geometry_type"]);

    if (model["name"])
        m_name = model["name"].as<std::string>();

    if (model["data_path"]) {
        ChAssertAlways(model["data_path"]["type"]);
        m_data_path = ReadDataPathType(model["data_path"]["type"]);
        if (model["data_path"]["root"])
            m_rel_path = model["data_path"]["root"].as<std::string>();
    }

    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserSphYAML] Loading Chrono::SPH model specification from: '" << yaml_filename << "'\n" << endl;
        cout << "model name: '" << m_name << "'" << endl;
        switch (m_data_path) {
            case DataPathType::ABS:
                cout << "using absolute file paths" << endl;
                break;
            case DataPathType::REL:
                cout << "using file paths relative to: '" << m_rel_path << "'" << endl;
                break;
        }
    }

    // Read fluid properties
    if (model["fluid_properties"]) {
        auto a = model["fluid properties"];
        if (a["density"])
            m_fluid.fluid_props.density = a["density"].as<double>();
        if (a["viscosity"])
            m_fluid.fluid_props.viscosity = a["viscosity"].as<double>();
        if (a["char_length"])
            m_fluid.fluid_props.char_length = a["char_length"].as<double>();
    }

    // Read soil properties
    if (model["soil_properties"]) {
        auto a = model["soil properties"];
        if (a["density"])
            m_fluid.soil_props.density = a["density"].as<double>();
        if (a["Young_modulus"])
            m_fluid.soil_props.Young_modulus = a["Young_modulus"].as<double>();
        if (a["Poisson_ratio"])
            m_fluid.soil_props.Poisson_ratio = a["Poisson_ratio"].as<double>();
        if (a["mu_I0"])
            m_fluid.soil_props.mu_I0 = a["mu_I0"].as<double>();
        if (a["mu_fric_s"])
            m_fluid.soil_props.mu_fric_s = a["mu_fric_s"].as<double>();
        if (a["mu_fric_2"])
            m_fluid.soil_props.mu_fric_2 = a["mu_fric_2"].as<double>();
        if (a["average_diam"])
            m_fluid.soil_props.average_diam = a["average_diam"].as<double>();
        if (a["cohesion_coefficient"])
            m_fluid.soil_props.cohesion_coeff = a["cohesion_coefficient"].as<double>();
    }

    // Read SPH state initialization settings
    if (model["initial_states"]) {
        auto a = model["initial_states"];
        if (a["depth_based_pressure"]) {
            m_depth_based_pressure = true;
            ChAssertAlways(a["zero_height"]);
            m_zero_height = a["zero_height"].as<double>();
            cout << "--------------   zero height " << m_zero_height << endl;
        }
        if (a["initial_velocity"]) {
            m_initial_velocity = true;
            m_velocity = ChParserMbsYAML::ReadVector(a["initial_velocity"]);
            cout << "------------  initi vel " << m_velocity << endl;
        }
    }

    // Read fluid domain settings
    bool has_walls = false;
    ChAssertAlways(model["fluid_domain"]);
    {
        auto a = model["fluid_domain"];
        switch (m_geometry_type) {
            case GeometryType::CARTESIAN:
                ChAssertAlways(a["dimensions"]);
                m_fluid.fluid_domain_cartesian = chrono_types::make_unique<BoxDomain>();
                m_fluid.fluid_domain_cartesian->dimensions = ChParserMbsYAML::ReadVector(a["dimensions"]);
                if (a["box_origin"]) {
                    m_fluid.fluid_domain_cartesian->origin = ChParserMbsYAML::ReadVector(a["box_origin"]);
                } else {
                    m_fluid.fluid_domain_cartesian->origin = VNULL;
                }
                if (a["box_walls"]) {
                    m_fluid.fluid_domain_cartesian->wall_code = ReadWallFlagsCartesian(a["box_walls"]);
                    has_walls = (m_fluid.fluid_domain_cartesian->wall_code != fsi::sph::BoxSide::NONE);
                } else {
                    m_fluid.fluid_domain_cartesian->wall_code = fsi::sph::BoxSide::NONE;
                }
                break;
            case GeometryType::CYLINDRICAL:
                ChAssertAlways(a["inner_radius"]);
                ChAssertAlways(a["outer_radius"]);
                ChAssertAlways(a["height"]);
                m_fluid.fluid_domain_cylindrical = chrono_types::make_unique<AnnulusDomain>();
                m_fluid.fluid_domain_cylindrical->inner_radius = a["inner_radius"].as<double>();
                m_fluid.fluid_domain_cylindrical->outer_radius = a["outer_radius"].as<double>();
                m_fluid.fluid_domain_cylindrical->height = a["height"].as<double>();
                if (a["cyl_origin"]) {
                    m_fluid.fluid_domain_cylindrical->origin = ChParserMbsYAML::ReadVector(a["cyl_origin"]);
                } else {
                    m_fluid.fluid_domain_cylindrical->origin = VNULL;
                }
                if (a["cyl_walls"]) {
                    m_fluid.fluid_domain_cylindrical->wall_code = ReadWallFlagsCylindrical(a["cyl_walls"]);
                    has_walls = (m_fluid.fluid_domain_cylindrical->wall_code != fsi::sph::CylSide::NONE);
                } else {
                    m_fluid.fluid_domain_cylindrical->wall_code = fsi::sph::CylSide::NONE;
                }
                break;
        }
    }

    // Read optional container settings
    if (model["container"]) {
        // Note: A container definition is not allowed if wall boundaries have already been defined!
        ChAssertAlways(!has_walls);

        auto a = model["container"];
        switch (m_geometry_type) {
            case GeometryType::CARTESIAN:
                m_fluid.container_cartesian = chrono_types::make_unique<BoxDomain>();
                ChAssertAlways(a["dimensions"]);
                m_fluid.container_cartesian->dimensions = ChParserMbsYAML::ReadVector(a["dimensions"]);
                if (a["box_origin"]) {
                    m_fluid.container_cartesian->origin = ChParserMbsYAML::ReadVector(a["box_origin"]);
                } else {
                    m_fluid.container_cartesian->origin = VNULL;
                }
                if (a["box_walls"]) {
                    m_fluid.container_cartesian->wall_code = ReadWallFlagsCartesian(a["box_walls"]);
                    has_walls = (m_fluid.container_cartesian->wall_code != fsi::sph::BoxSide::NONE);
                } else {
                    m_fluid.container_cartesian->wall_code = fsi::sph::BoxSide::NONE;
                }
                break;
            case GeometryType::CYLINDRICAL:
                m_fluid.fluid_domain_cylindrical = chrono_types::make_unique<AnnulusDomain>();
                ChAssertAlways(a["inner_radius"]);
                ChAssertAlways(a["outer_radius"]);
                ChAssertAlways(a["height"]);
                m_fluid.fluid_domain_cylindrical->inner_radius = a["inner_radius"].as<double>();
                m_fluid.fluid_domain_cylindrical->outer_radius = a["outer_radius"].as<double>();
                m_fluid.fluid_domain_cylindrical->height = a["height"].as<double>();
                if (a["cyl_origin"]) {
                    m_fluid.fluid_domain_cylindrical->origin = ChParserMbsYAML::ReadVector(a["cyl_origin"]);
                } else {
                    m_fluid.fluid_domain_cylindrical->origin = VNULL;
                }
                if (a["cyl_walls"]) {
                    m_fluid.fluid_domain_cylindrical->wall_code = ReadWallFlagsCylindrical(a["cyl_walls"]);
                    has_walls = (m_fluid.fluid_domain_cylindrical->wall_code != fsi::sph::CylSide::NONE);
                } else {
                    m_fluid.fluid_domain_cylindrical->wall_code = fsi::sph::CylSide::NONE;
                }
                break;
        }
    }

    // Read optional computational domain settings
    if (model["computational_domain"]) {
        auto a = model["computational_domain"];
        m_fluid.computational_domain = chrono_types::make_unique<ComputationalDomain>();
        ChAssertAlways(a["aabb_min"]);
        ChAssertAlways(a["aabb_max"]);
        m_fluid.computational_domain->aabb.min = ChParserMbsYAML::ReadVector(a["aabb_min"]);
        m_fluid.computational_domain->aabb.max = ChParserMbsYAML::ReadVector(a["aabb_max"]);
        if (a["x_bc_type"]) {
            auto x_bc_type = ReadBoundaryConditionType(a["x_bc_type"]);
            if (x_bc_type != fsi::sph::BCType::NONE && m_geometry_type == GeometryType::CARTESIAN) {
                ChAssertAlways(m_fluid.fluid_domain_cartesian->wall_code &
                               static_cast<int>(fsi::sph::BoxSide::X_NEG) == 0);
                ChAssertAlways(m_fluid.fluid_domain_cartesian->wall_code &
                               static_cast<int>(fsi::sph::BoxSide::X_POS) == 0);
            }
            m_fluid.computational_domain->bc_type.x = x_bc_type;
        }
        if (a["y_bc_type"]) {
            auto y_bc_type = ReadBoundaryConditionType(a["y_bc_type"]);
            if (y_bc_type != fsi::sph::BCType::NONE && m_geometry_type == GeometryType::CARTESIAN) {
                ChAssertAlways(m_fluid.fluid_domain_cartesian->wall_code &
                               static_cast<int>(fsi::sph::BoxSide::Y_NEG) == 0);
                ChAssertAlways(m_fluid.fluid_domain_cartesian->wall_code &
                               static_cast<int>(fsi::sph::BoxSide::Y_POS) == 0);
            }
            m_fluid.computational_domain->bc_type.y = y_bc_type;
        }
        if (a["z_bc_type"]) {
            auto z_bc_type = ReadBoundaryConditionType(a["z_bc_type"]);
            if (z_bc_type != fsi::sph::BCType::NONE && m_geometry_type == GeometryType::CARTESIAN) {
                ChAssertAlways(m_fluid.fluid_domain_cartesian->wall_code &
                               static_cast<int>(fsi::sph::BoxSide::Z_NEG) == 0);
                ChAssertAlways(m_fluid.fluid_domain_cartesian->wall_code &
                               static_cast<int>(fsi::sph::BoxSide::Z_POS) == 0);
            }
            m_fluid.computational_domain->bc_type.z = z_bc_type;
        }
    }

    if (m_verbose) {
        cout << endl;
        m_fluid.PrintInfo();
    }

    m_model_loaded = true;
}

// -----------------------------------------------------------------------------

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


std::shared_ptr<fsi::sph::ChFsiProblemSPH> ChParserSphYAML::CreateFsiProblemSPH(bool initialize) {
    if (m_verbose) {
        cout << "\n-------------------------------------------------" << endl;
        cout << "\n[ChParserSphYAML] Create ChFSIProblemSPH\n" << endl;
    }

    if (!m_model_loaded) {
        cerr << "[ChParserSphYAML::CreateFsiProblemSPH] Error: no YAML model file loaded." << endl;
        throw std::runtime_error("No YAML model file loaded");
    }

    if (!m_sim_loaded) {
        cerr << "[ChParserSphYAML::CreateFsiProblemSPH] Error: no YAML simulation file loaded." << endl;
        throw std::runtime_error("No YAML simulation file loaded");
    }

    // Create a Chrono FSI SPH problem of specified type with no MBS attached
    switch (m_geometry_type) {
        case GeometryType::CARTESIAN:
            m_fsi_problem = chrono_types::make_shared<fsi::sph::ChFsiProblemCartesian>(m_sim.sph.initial_spacing);
            break;
        case GeometryType::CYLINDRICAL:
            m_fsi_problem = chrono_types::make_shared<fsi::sph::ChFsiProblemCylindrical>(m_sim.sph.initial_spacing);
            break;
    }

    m_fsi_problem->SetVerbose(m_verbose);

    // Set simulation parameters
    m_fsi_problem->SetGravitationalAcceleration(m_sim.gravity);
    m_fsi_problem->SetStepSizeCFD(m_sim.time_step);
    m_fsi_problem->SetSPHParameters(m_sim.sph);

    // Set fluid properties
    switch (m_fluid.physics_problem) {
        case fsi::sph::PhysicsProblem::CFD:
            m_fsi_problem->SetCfdSPH(m_fluid.fluid_props);
            break;
        case fsi::sph::PhysicsProblem::CRM:
            m_fsi_problem->SetElasticSPH(m_fluid.soil_props);
            break;
    }

    // Set callback for initial states
    if (m_depth_based_pressure || m_initial_velocity) {
        m_fsi_problem->RegisterParticlePropertiesCallback(
            chrono_types::make_shared<SPHPropertiesCallback>(m_depth_based_pressure, m_zero_height, m_initial_velocity, m_velocity));
    }

    // Create fluid domain and optional container
    switch (m_geometry_type) {
        case GeometryType::CARTESIAN: {
            auto fsi_problem = std::static_pointer_cast<fsi::sph::ChFsiProblemCartesian>(m_fsi_problem);
            fsi_problem->Construct(m_fluid.fluid_domain_cartesian->dimensions,  //
                                   m_fluid.fluid_domain_cartesian->origin,      //
                                   m_fluid.fluid_domain_cartesian->wall_code);
            if (m_fluid.container_cartesian) {
                fsi_problem->AddBoxContainer(m_fluid.container_cartesian->dimensions,  //
                                             m_fluid.container_cartesian->origin,      //
                                             m_fluid.container_cartesian->wall_code);
            }

            break;
        }
        case GeometryType::CYLINDRICAL: {
            auto fsi_problem = std::static_pointer_cast<fsi::sph::ChFsiProblemCylindrical>(m_fsi_problem);
            fsi_problem->Construct(m_fluid.fluid_domain_cylindrical->inner_radius,  //
                                   m_fluid.fluid_domain_cylindrical->outer_radius,  //
                                   m_fluid.fluid_domain_cylindrical->height,        //
                                   m_fluid.fluid_domain_cylindrical->origin,        //
                                   m_fluid.fluid_domain_cylindrical->wall_code);
            if (m_fluid.container_cylindrical) {
                fsi_problem->AddCylindricalContainer(m_fluid.container_cylindrical->inner_radius,  //
                                                     m_fluid.container_cylindrical->outer_radius,  //
                                                     m_fluid.container_cylindrical->height,        //
                                                     m_fluid.container_cylindrical->origin,        //
                                                     m_fluid.container_cylindrical->wall_code);
            }

            break;
        }
    }

    // Explicitly set computational domain (if provided)
    if (m_fluid.computational_domain) {
        m_fsi_problem->SetComputationalDomain(m_fluid.computational_domain->aabb,
                                              m_fluid.computational_domain->bc_type);
    }

    // Initialize FSI problem
    if (initialize)
        m_fsi_problem->Initialize();

    return m_fsi_problem;
}

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
std::shared_ptr<vsg3d::ChVisualSystemVSGPlugin> ChParserSphYAML::GetVisualizationPlugin() const {
    auto vis = chrono_types::make_shared<fsi::sph::ChSphVisualizationVSG>(m_fsi_problem->GetFsiSystemSPH().get());

    vis->EnableFluidMarkers(m_sim.visualization.sph_markers);
    vis->EnableBoundaryMarkers(m_sim.visualization.bndry_bce_markers);
    vis->EnableRigidBodyMarkers(m_sim.visualization.rigid_bce_markers);

    if (m_sim.visualization.color_callback)
        vis->SetSPHColorCallback(m_sim.visualization.color_callback, m_sim.visualization.colormap);
    if (m_sim.visualization.visibility_callback_sph)
        vis->SetSPHVisibilityCallback(m_sim.visualization.visibility_callback_sph);
    if (m_sim.visualization.visibility_callback_bce)
        vis->SetBCEVisibilityCallback(m_sim.visualization.visibility_callback_bce);
    return vis;
}
#endif

// -----------------------------------------------------------------------------

void ChParserSphYAML::SaveOutput(int frame) {
    if (m_sim.output.type == ChOutput::Type::NONE)
        return;

    // Create the output DB if needed
    if (!m_output_db) {
        std::string filename = m_sim.output.dir + "/" + m_name;
        if (!m_output_dir.empty())
            filename = m_output_dir + "/" + filename;
        if (m_verbose) {
            cout << "\n-------------------------------------------------" << endl;
            cout << "\nOutput file: " << filename << endl;
        }
        switch (m_sim.output.type) {
            case ChOutput::Type::ASCII:
                m_output_db = chrono_types::make_shared<ChOutputASCII>(filename + ".txt");
                break;
            case ChOutput::Type::HDF5:
#ifdef CHRONO_HAS_HDF5
                m_output_db = chrono_types::make_shared<ChOutputHDF5>(filename + ".h5", m_sim.output.mode);
                break;
#else
                return;
#endif
        }
        m_output_db->Initialize();
    }

    //// TODO
}

// -----------------------------------------------------------------------------

ChParserSphYAML::FluidParams::FluidParams() {}

void ChParserSphYAML::FluidParams::PrintInfo() {
    switch (physics_problem) {
        case fsi::sph::PhysicsProblem::CFD:
            cout << "fluid parameters" << endl;
            cout << "  properties" << endl;
            cout << "     density:               " << fluid_props.density << endl;
            cout << "     viscosity:             " << fluid_props.viscosity << endl;
            cout << "     characteristic length: " << fluid_props.char_length << endl;
            break;
        case fsi::sph::PhysicsProblem::CRM:
            cout << "soil parameters" << endl;
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

ChParserSphYAML::VisParams::VisParams()
    : render(false),
      use_splashsurf(false),
      sph_markers(true),
      rigid_bce_markers(true),
      flex_bce_markers(true),
      bndry_bce_markers(false),
      active_boxes(false),
      colormap(ChColormap::Type::FAST),
      write_images(false),
      image_dir(".") {}

ChParserSphYAML::OutputParameters::OutputParameters()
    : type(ChOutput::Type::NONE), mode(ChOutput::Mode::FRAMES), fps(100), dir(".") {}

ChParserSphYAML::SimParams::SimParams() : gravity({0, 0, -9.8}), time_step(1e-4), end_time(-1) {}

void ChParserSphYAML::SimParams::PrintInfo() {
    cout << "simulation end time:        " << (end_time < 0 ? "infinite" : std::to_string(end_time)) << endl;
    cout << "integration time step:      " << time_step << endl;
    cout << "gravitational acceleration: " << gravity << endl;
    cout << endl;

    cout << "SPH settings" << endl;

    //// TODO

    cout << endl;
    visualization.PrintInfo();
    cout << endl;
    output.PrintInfo();
}

void ChParserSphYAML::VisParams::PrintInfo() {
    if (!render) {
        cout << "no run-time visualization" << endl;
        return;
    }

    cout << "run-time visualization" << endl;
    cout << "  render SPH particles:       " << sph_markers << endl;
    cout << "  render BCE boundary:        " << bndry_bce_markers << endl;
    cout << "  render BCE rigid solids:    " << rigid_bce_markers << endl;
    cout << "  render BCE flexible colids: " << flex_bce_markers << endl;
    cout << "  render active boxes:        " << active_boxes << endl;
#ifdef CHRONO_VSG
    if (use_splashsurf) {
        cout << "  splashsurf parameters" << endl;
        cout << "    smoothing length used for the SPH kernel: " << splashsurf_params->smoothing_length << endl;
        cout << "    cube edge length used for marching cubes: " << splashsurf_params->cube_size << endl;
        cout << "    iso-surface threshold for the density:    " << splashsurf_params->surface_threshold << endl;
    }
#endif
}

void ChParserSphYAML::OutputParameters::PrintInfo() {
    if (type == ChOutput::Type::NONE) {
        cout << "no output" << endl;
        return;
    }

    cout << "output" << endl;
    cout << "  type:                 " << ChOutput::GetOutputTypeAsString(type) << endl;
    cout << "  mode:                 " << ChOutput::GetOutputModeAsString(mode) << endl;
    cout << "  output FPS:           " << fps << endl;
    cout << "  outut directory:      " << dir << endl;
}

// =============================================================================

static void PrintNodeType(const YAML::Node& node) {
    switch (node.Type()) {
        case YAML::NodeType::Null:
            cout << " Null" << endl;
            break;
        case YAML::NodeType::Scalar:
            cout << " Scalar" << endl;
            break;
        case YAML::NodeType::Sequence:
            cout << " Sequence" << endl;
            break;
        case YAML::NodeType::Map:
            cout << " Map" << endl;
            break;
        case YAML::NodeType::Undefined:
            cout << " Undefined" << endl;
            break;
    }
}

std::string ChParserSphYAML::GetDatafilePath(const std::string& filename) {
    std::string full_filename = "";
    switch (m_data_path) {
        case DataPathType::ABS:
            full_filename = filename;
            break;
        case DataPathType::REL:
            full_filename = m_script_directory + "/" + m_rel_path + "/" + filename;
            break;
    }

    cout << "File: " << full_filename << endl;
    auto filepath = filesystem::path(full_filename);

    ChAssertAlways(filepath.exists());
    ChAssertAlways(filepath.is_file());

    return full_filename;
}

// -----------------------------------------------------------------------------

fsi::sph::PhysicsProblem ChParserSphYAML::ReadPhysicsProblemType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "CFD")
        return fsi::sph::PhysicsProblem::CFD;
    if (val == "CRM")
        return fsi::sph::PhysicsProblem::CRM;
    return fsi::sph::PhysicsProblem::CFD;
}

ChParserSphYAML::GeometryType ChParserSphYAML::ReadGeometryType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "CARTESIAN")
        return GeometryType::CARTESIAN;
    if (val == "CYLINDRICAL")
        return GeometryType::CYLINDRICAL;
    return GeometryType::CARTESIAN;
}

ChParserSphYAML::DataPathType ChParserSphYAML::ReadDataPathType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "RELATIVE")
        return DataPathType::REL;
    else
        return DataPathType::ABS;
}

fsi::sph::EosType ChParserSphYAML::ReadEosType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "ISOTHERMAL")
        return fsi::sph::EosType::ISOTHERMAL;
    if (val == "TAIT")
        return fsi::sph::EosType::TAIT;
    return fsi::sph::EosType::ISOTHERMAL;
}

fsi::sph::KernelType ChParserSphYAML::ReadKernelType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
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
    auto val = ToUpper(a.as<std::string>());
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
    auto val = ToUpper(a.as<std::string>());
    if (val == "ADAMI")
        return fsi::sph::BoundaryMethod::ADAMI;
    if (val == "HOLMES")
        return fsi::sph::BoundaryMethod::HOLMES;
    return fsi::sph::BoundaryMethod::ADAMI;
}

fsi::sph::ShiftingMethod ChParserSphYAML::ReadShiftingMethod(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
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
    auto val = ToUpper(a.as<std::string>());
    if (val == "LAMINAR")
        return fsi::sph::ViscosityMethod::LAMINAR;
    if (val == "ARTIFICIAL_UNILATERAL")
        return fsi::sph::ViscosityMethod::ARTIFICIAL_UNILATERAL;
    if (val == "ARTIFICIAL_BILATERAL")
        return fsi::sph::ViscosityMethod::ARTIFICIAL_BILATERAL;
    return fsi::sph::ViscosityMethod::ARTIFICIAL_UNILATERAL;
}

ChOutput::Type ChParserSphYAML::ReadOutputType(const YAML::Node& a) {
    auto type = ToUpper(a.as<std::string>());
    if (type == "ASCII")
        return ChOutput::Type::ASCII;
    if (type == "HDF5")
        return ChOutput::Type::HDF5;
    return ChOutput::Type::NONE;
}

ChOutput::Mode ChParserSphYAML::ReadOutputMode(const YAML::Node& a) {
    auto mode = ToUpper(a.as<std::string>());
    if (mode == "SERIES")
        return ChOutput::Mode::SERIES;
    if (mode == "FRAMES")
        return ChOutput::Mode::FRAMES;
    return ChOutput::Mode::FRAMES;
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
    auto val = ToUpper(a.as<std::string>());
    if (val == "NONE")
        return fsi::sph::BCType::NONE;
    if (val == "PERIODIC")
        return fsi::sph::BCType::PERIODIC;
    if (val == "INLET_OUTLET")
        return fsi::sph::BCType::INLET_OUTLET;
    return fsi::sph::BCType::NONE;
}

ChParserSphYAML::ParticleColoringType ChParserSphYAML::ReadParticleColoringType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "NONE")
        return ParticleColoringType::NONE;
    if (val == "HEIGHT")
        return ParticleColoringType::HEIGHT;
    if (val == "VELOCITY")
        return ParticleColoringType::VELOCITY;
    if (val == "DENSITY")
        return ParticleColoringType::DENSITY;
    if (val == "PRESSURE")
        return ParticleColoringType::PRESSURE;
    return ParticleColoringType::NONE;
}

ChColormap::Type ChParserSphYAML::ReadColorMapType(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "BLACK_BODY")
        return ChColormap::Type::BLACK_BODY;
    if (val == "BROWN")
        return ChColormap::Type::BROWN;
    if (val == "COPPER")
        return ChColormap::Type::COPPER;
    if (val == "FAST")
        return ChColormap::Type::FAST;
    if (val == "INFERNO")
        return ChColormap::Type::INFERNO;
    if (val == "JET")
        return ChColormap::Type::JET;
    if (val == "KINDLMANN")
        return ChColormap::Type::KINDLMANN;
    if (val == "BLACK_BODY")
        return ChColormap::Type::BLACK_BODY;
    if (val == "PLASMA")
        return ChColormap::Type::PLASMA;
    if (val == "RED_BLUE")
        return ChColormap::Type::RED_BLUE;
    return ChColormap::Type::JET;
}

fsi::sph::MarkerPlanesVisibilityCallback::Mode ChParserSphYAML::ReadVisibilityMode(const YAML::Node& a) {
    auto val = ToUpper(a.as<std::string>());
    if (val == "ANY")
        return fsi::sph::MarkerPlanesVisibilityCallback::Mode::ANY;
    if (val == "ALL")
        return fsi::sph::MarkerPlanesVisibilityCallback::Mode::ALL;
    return fsi::sph::MarkerPlanesVisibilityCallback::Mode::ALL;
}

}  // namespace parsers
}  // namespace chrono
