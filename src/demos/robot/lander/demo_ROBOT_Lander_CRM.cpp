// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Huzaifa Mustafa Unjhawala
// =============================================================================
//
// Demo code for a lunar lander simulation
// - Simple cylinder body with 4 rigidly attached legs
// - Drops onto a rigid platform under lunar gravity
// - Uses NSC contact model with Bullet collision detection
// - Visualized with VSG
//
// =============================================================================

#include <iomanip>
#include <sstream>
#include <fstream>
#include <algorithm>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChContactMaterialNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiProblemSPH.h"
#include "chrono_fsi/sph/ChFsiDefinitionsSPH.h"

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#ifdef CHRONO_VSG
    #include "chrono_vsg/ChVisualSystemVSG.h"
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
using namespace chrono::vsg3d;
#endif

#include "demos/SetChronoSolver.h"

#include "model/Lander.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Lunar gravity acceleration (m/s^2)
const double LUNAR_GRAVITY = 1.62;
const double EARTH_GRAVITY = 9.81;
const double MARS_GRAVITY = 3.71;

// Platform parameters
const double TERRAIN_SIZE_X = 6.0;  // meters
const double TERRAIN_SIZE_Y = 6.0;  // meters
const double TERRAIN_SIZE_Z = 0.3;  // meters

// Footpad parameters
const double FOOTPAD_DIAMETER = 0.5;
const double FOOTPAD_HEIGHT = 0.1;
const double FOOTPAD_OFFSET = 0.02;
const double FOOTPAD_MASS = 2.0;

// Drop height (2m above the legs) - used to calculate initial velocity if USE_DROP_HEIGHT_MODE is true
const double DROP_HEIGHT = 2.0;

// Initial velocity mode switch: true = calculate from drop height (v = sqrt(2*g*h)), false = use INITIAL_VELOCITY
// directly
const bool USE_DROP_HEIGHT_MODE = false;  // Set to false to use INITIAL_VELOCITY directly

// Initial velocity (m/s) - used when USE_DROP_HEIGHT_MODE is false (negative = downward)
const double INITIAL_VELOCITY = -1.0;  // m/s downward

// Clearance above ground when starting simulation (m)
const double GROUND_CLEARANCE = 0.05;  // meters
const double t_end = 1.2;
const bool USE_ACTIVE_DOMAIN = true;

// Output parameters
const double output_fps = 100.0;  // Output FPS for CSV and snapshots
const bool snapshots = true;      // Enable snapshot saving

const bool ADD_LANDERBODY_BCE = true;
const bool ADD_LANDERLEGS_BCE = true;

class SPHPropertiesCallbackWithPressureScale : public ChFsiProblemSPH::ParticlePropertiesCallback {
  public:
    SPHPropertiesCallbackWithPressureScale(double zero_height, double pre_pressure_scale)
        : ParticlePropertiesCallback(), zero_height(zero_height), pre_pressure_scale(pre_pressure_scale) {}

    virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override {
        double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
        p0 = sysSPH.GetDensity() * gz * (zero_height - pos.z());
        rho0 = sysSPH.GetDensity();
        mu0 = sysSPH.GetViscosity();
        v0 = ChVector3d(0, 0, 0);
        pre_pressure_scale0 = pre_pressure_scale;
    }

    double zero_height;
    double pre_pressure_scale;
};

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Parse command line arguments
    ChCLI cli(argv[0], "Lunar Lander on CRM Terrain Demo");

    // Default values
    std::string rheology_model_crm = "MU_OF_I";  // "MU_OF_I" or "MCC"
    double kappa = 0.01;
    double lambda = 0.04;
    double pre_pressure_scale = 2.0;
    bool no_vis = false;
    double gravity_polar_deg = 0.0;
    double gravity_azimuth_deg = 0.0;
    double gravity_magnitude = LUNAR_GRAVITY;

    cli.AddOption<std::string>("Physics", "rheology_model_crm", "Rheology model (MU_OF_I/MCC)", rheology_model_crm);
    cli.AddOption<double>("Physics", "pre_pressure_scale", "Pre-pressure scale", std::to_string(pre_pressure_scale));
    cli.AddOption<double>("Physics", "kappa", "kappa", std::to_string(kappa));
    cli.AddOption<double>("Physics", "lambda", "lambda", std::to_string(lambda));
    cli.AddOption<bool>("Visualization", "no_vis", "Disable visualization", "false");
    cli.AddOption<double>("Physics", "gravity_polar_deg", "Gravity polar angle (degrees)",
                          std::to_string(gravity_polar_deg));
    cli.AddOption<double>("Physics", "gravity_azimuth_deg", "Gravity azimuth angle (degrees)",
                          std::to_string(gravity_azimuth_deg));
    std::string gravity_planet = "moon";
    cli.AddOption<std::string>("Physics", "gravity_planet", "Gravity planet (earth/mars/moon)", gravity_planet);
    if (!cli.Parse(argc, argv))
        return 1;

    rheology_model_crm = cli.GetAsType<std::string>("rheology_model_crm");
    pre_pressure_scale = cli.GetAsType<double>("pre_pressure_scale");
    kappa = cli.GetAsType<double>("kappa");
    lambda = cli.GetAsType<double>("lambda");
    no_vis = cli.GetAsType<bool>("no_vis");
    gravity_polar_deg = cli.GetAsType<double>("gravity_polar_deg");
    gravity_azimuth_deg = cli.GetAsType<double>("gravity_azimuth_deg");
    gravity_planet = cli.GetAsType<std::string>("gravity_planet");
    bool enable_vis = !no_vis;

    // Convert to lowercase for case-insensitive comparison
    std::transform(gravity_planet.begin(), gravity_planet.end(), gravity_planet.begin(), ::tolower);

    if (gravity_planet == "earth") {
        gravity_magnitude = EARTH_GRAVITY;
    } else if (gravity_planet == "mars") {
        gravity_magnitude = MARS_GRAVITY;
    } else if (gravity_planet == "moon") {
        gravity_magnitude = LUNAR_GRAVITY;
    }

    // Create the physical system
    ChSystemNSC sys;
    // ChSystemSMC sys;
    // Simulation loop
    double step_size = 5e-4;
    double mu_s = 0.6;
    double initial_spacing = 0.01;
    ChVector3d active_box_dim(FOOTPAD_DIAMETER * 4.0, FOOTPAD_DIAMETER * 4.0, FOOTPAD_HEIGHT * 10.0);
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    SetChronoSolver(sys, ChSolver::Type::BARZILAIBORWEIN, ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    // Set lunar gravity (downward in Z direction)
    double gravity_x = -gravity_magnitude * std::sin(gravity_polar_deg * CH_PI / 180.0) *
                       std::cos(gravity_azimuth_deg * CH_PI / 180.0);
    double gravity_y = -gravity_magnitude * std::sin(gravity_polar_deg * CH_PI / 180.0) *
                       std::sin(gravity_azimuth_deg * CH_PI / 180.0);
    double gravity_z = -gravity_magnitude * std::cos(gravity_polar_deg * CH_PI / 180.0);
    sys.SetGravitationalAcceleration(ChVector3d(gravity_x, gravity_y, gravity_z));

    // Create contact material
    auto contact_material = chrono_types::make_shared<ChContactMaterialNSC>();
    contact_material->SetFriction(0.7f);
    contact_material->SetRestitution(0.05f);

    // =============================================================================
    // Create the lander using the Lander class
    // =============================================================================
    Lander lander(&sys, contact_material);

    // Position the lander: place it just above the ground (GROUND_CLEARANCE)
    // The lowest point (footpad or leg end) will be at platform_top + GROUND_CLEARANCE
    double platform_top = TERRAIN_SIZE_Z;
    ChVector3d lowest_point_pos(0, 0, platform_top);
    ChFrame<> lander_pos(lowest_point_pos, QUNIT);
    lander.SetFootpadParameters(FOOTPAD_DIAMETER, FOOTPAD_HEIGHT, FOOTPAD_OFFSET, FOOTPAD_MASS);
    lander.SetUseFootpads(true);
    lander.SetUseSphericalJoint(false);

    // Optionally configure other lander parameters (using defaults from demo if not set)
    // lander.SetCylinderParameters(4.0, 1.0, 2000.0);
    // lander.SetLegParameters(1.5, 0.05, CH_PI / 6.0, 0.6, 10.0);

    // Initialize the lander at the specified position
    lander.Initialize(lander_pos, GROUND_CLEARANCE);

    // =============================================================================
    // Set initial velocity to simulate drop from height
    // =============================================================================
    if (USE_DROP_HEIGHT_MODE) {
        // Calculate velocity from drop height: v = sqrt(2 * g * h)
        lander.SetInitialVelocityFromDropHeight(DROP_HEIGHT, LUNAR_GRAVITY);
    } else {
        // Use direct velocity value (should be negative for downward)
        lander.SetInitialVelocity(ChVector3d(0, 0, INITIAL_VELOCITY));
    }

    // ================================================================
    // Create the FSI problem
    // ================================================================
    ChFsiProblemCartesian fsi(initial_spacing, &sys);
    fsi.SetVerbose(true);
    auto sysFSI = fsi.GetFsiSystemSPH();

    // Set gravitational acceleration
    fsi.SetGravitationalAcceleration(ChVector3d(gravity_x, gravity_y, gravity_z));

    // Set integration step size
    fsi.SetStepSizeCFD(step_size);
    fsi.SetStepsizeMBD(step_size);

    // Set soil propertiees
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1700;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    if (rheology_model_crm == "MU_OF_I") {
        mat_props.rheology_model = RheologyCRM::MU_OF_I;
        mat_props.mu_I0 = 0.04;
        mat_props.mu_fric_s = mu_s;
        mat_props.mu_fric_2 = mu_s;
        mat_props.average_diam = 0.005;
        mat_props.cohesion_coeff = 0;  // default
    } else {
        mat_props.rheology_model = RheologyCRM::MCC;
        double angle_mus = std::atan(mu_s);
        mat_props.mcc_M = (6 * std::sin(angle_mus)) / (3 - std::sin(angle_mus));
        std::cout << "MCC M: " << mat_props.mcc_M << std::endl;
        mat_props.mcc_kappa = kappa;
        mat_props.mcc_lambda = lambda;
    }
    fsi.SetElasticSPH(mat_props);

    // Set SPH solution parameters
    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.num_bce_layers = 3;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = initial_spacing;
    sph_params.d0_multiplier = 1.3;
    sph_params.artificial_viscosity = 0.2;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;

    sph_params.kernel_type = KernelType::WENDLAND;
    sph_params.boundary_method = BoundaryMethod::HOLMES;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 2.0;
    sph_params.num_proximity_search_steps = 1;
    sph_params.use_variable_time_step = true;
    sph_params.use_delta_sph = true;
    fsi.SetSPHParameters(sph_params);

    fsi.RegisterParticlePropertiesCallback(
        chrono_types::make_shared<SPHPropertiesCallbackWithPressureScale>(TERRAIN_SIZE_Z, pre_pressure_scale));

    // Add the footpads as FSI bodies
    auto footpads = lander.GetFootpads();

    // This is pretty inconvient
    auto geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
    geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(ChVector3d(0, 0, 0), ChVector3d(0, 0, 1),
                                                                            FOOTPAD_DIAMETER / 2.0, FOOTPAD_HEIGHT));
    for (auto& footpad : footpads) {
        fsi.AddRigidBody(footpad, geometry, false);
    }
    if (ADD_LANDERLEGS_BCE) {
        auto leg_length = lander.GetLegLength();
        auto leg_radius = lander.GetLegRadius();
        auto leg_geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        leg_geometry->coll_cylinders.push_back(
            utils::ChBodyGeometry::CylinderShape(ChVector3d(0, 0, 0), ChVector3d(0, 0, 1), leg_radius, leg_length));
        auto legs = lander.GetLegs();
        for (auto& leg : legs) {
            fsi.AddRigidBody(leg, leg_geometry, false);
        }
    }
    if (ADD_LANDERBODY_BCE) {
        auto cylinder_length = lander.GetCylinderLength();
        auto cylinder_radius = lander.GetCylinderRadius();
        auto cylinder_geometry = chrono_types::make_shared<utils::ChBodyGeometry>();
        cylinder_geometry->coll_cylinders.push_back(utils::ChBodyGeometry::CylinderShape(
            ChVector3d(0, 0, 0), ChVector3d(0, 0, 1), cylinder_radius, cylinder_length));
        auto lander_body = lander.GetBody();
        fsi.AddRigidBody(lander_body, cylinder_geometry, false);
    }
    if (USE_ACTIVE_DOMAIN) {
        if (ADD_LANDERBODY_BCE) {
            // Make box bigger
            auto cylinder_length = lander.GetCylinderLength();
            active_box_dim.z() = cylinder_length;
            fsi.SetActiveDomain(active_box_dim);
        } else {
            fsi.SetActiveDomain(active_box_dim);
        }
    }

    std::cout << "Constructing terrain..." << std::endl;
    fsi.Construct(ChVector3d(TERRAIN_SIZE_X, TERRAIN_SIZE_Y, TERRAIN_SIZE_Z), ChVector3d(0, 0, 0), BoxSide::Z_NEG);

    // The function above may take very long to run (for a large terrain patch).
    // Optionally, use the Construct function above once and save the csv files at the first step. Then use:
    ////fsi.Construct(sph_file, bce_file, ChVector3d(0, 0, 0), false);

    fsi.Initialize();

    // =============================================================================
    // Create output directory
    // =============================================================================
    std::string base_dir = GetChronoOutputPath();
    filesystem::create_directory(filesystem::path(base_dir));

    // Create output directory name with rheology model and parameters
    std::stringstream ss;
    ss << std::fixed;
    ss << base_dir << "ROBOT_Lander_CRM";
    ss << "_" << rheology_model_crm;
    ss << "_gravity_planet_" << gravity_planet;
    ss << "_gravity_polar_deg_" << gravity_polar_deg;
    ss << "_gravity_azimuth_deg_" << gravity_azimuth_deg;
    if (rheology_model_crm == "MCC") {
        ss << "_pre_pressure_scale_" << std::setprecision(2) << pre_pressure_scale;
        ss << "_kappa_" << std::setprecision(2) << kappa;
        ss << "_lambda_" << std::setprecision(2) << lambda;
    }
    ss << "/";
    std::string out_dir = ss.str();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create snapshots directory if enabled and visualization is enabled
    if (snapshots && enable_vis) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "snapshots" << std::endl;
            return 1;
        }
    }

    // Create particles and fsi directories for marker output
    if (!filesystem::create_directory(filesystem::path(out_dir + "particles"))) {
        std::cerr << "Error creating directory " << out_dir + "particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "fsi" << std::endl;
        return 1;
    }

    // Get SPH system for saving particles and markers
    auto sysSPH = fsi.GetFluidSystemSPH();

    // Open CSV file for lander body data
    std::string csv_file = out_dir + "lander_body_data.csv";
    std::ofstream csv_output(csv_file);
    csv_output << "time,px,py,pz,vx,vy,vz,wx,wy,wz" << std::endl;

    // =============================================================================
    // Create VSG visualization
    // =============================================================================

    std::shared_ptr<ChVisualSystem> vis;

    if (enable_vis) {
#ifdef CHRONO_VSG
        // FSI plugin
        auto col_callback =
            chrono_types::make_shared<ParticleHeightColorCallback>(TERRAIN_SIZE_Z, TERRAIN_SIZE_Z + FOOTPAD_HEIGHT);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(sysFSI.get());
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetSPHColorCallback(col_callback);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sys);
        visVSG->SetWindowTitle("Lunar Lander Simulation");
        visVSG->SetWindowSize(ChVector2i(1920, 1080));
        visVSG->SetWindowPosition(100, 100);
        visVSG->SetCameraVertical(CameraVerticalDir::Z);
        visVSG->AddCamera(ChVector3d(6, -6, 6.0), ChVector3d(0, 0, TERRAIN_SIZE_Z + 2.0));
        visVSG->SetLightIntensity(1.0f);
        visVSG->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
        visVSG->SetBackgroundColor(ChColor(0.1f, 0.1f, 0.15f));  // Dark blue-gray (space-like)
        visVSG->EnableSkyBox();
        visVSG->Initialize();
        // Always enable active domain visualization after initialization
        if (USE_ACTIVE_DOMAIN) {
            visFSI->SetActiveBoxVisibility(false, -1);  // -1 means all active boxes
        }
        vis = visVSG;
#else
        std::cout << "VSG not available. Cannot visualize." << std::endl;
        return 1;
#endif
    }

    std::cout << "\nStarting simulation..." << std::endl;
    if (!enable_vis) {
        std::cout << "Visualization: DISABLED (--no_vis flag set)" << std::endl;
    }
    std::cout << "Lander mass: " << lander.GetLanderMass() << " kg" << std::endl;
    std::cout << "Cylinder: length=" << lander.GetCylinderLength() << " m, diameter=" << lander.GetCylinderDiameter()
              << " m" << std::endl;
    std::cout << "Lunar gravity: " << LUNAR_GRAVITY << " m/s^2" << std::endl;
    std::cout << "Initial position: " << GROUND_CLEARANCE << " m above platform" << std::endl;
    if (USE_DROP_HEIGHT_MODE) {
        std::cout << "Velocity mode: Calculated from drop height" << std::endl;
        std::cout << "Drop height: " << DROP_HEIGHT << " m (simulated)" << std::endl;
        double initial_velocity_z = -std::sqrt(2.0 * LUNAR_GRAVITY * DROP_HEIGHT);
        std::cout << "Initial velocity: " << initial_velocity_z << " m/s (downward)" << std::endl;
    } else {
        std::cout << "Velocity mode: Direct specification" << std::endl;
        std::cout << "Initial velocity: " << INITIAL_VELOCITY << " m/s (downward)" << std::endl;
    }
    std::cout << "Footpad joint type: "
              << (lander.GetUseSphericalJoint() ? "Spherical (rotating)" : "Rigid lock (fixed)") << std::endl;

    // Simulation loop variables
    double t = 0;
    int render_frame = 0;
    int csv_frame = 0;

    while (t < t_end) {
        // Write CSV data at output FPS
        if (t >= csv_frame / output_fps) {
            ChVector3d pos = lander.GetBody()->GetPos();
            ChVector3d vel = lander.GetBody()->GetPosDt();
            ChVector3d ang_vel = lander.GetBody()->GetAngVelParent();
            csv_output << t << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << vel.x() << "," << vel.y()
                       << "," << vel.z() << "," << ang_vel.x() << "," << ang_vel.y() << "," << ang_vel.z() << std::endl;
            csv_frame++;
            std::cout << "Time: " << sys.GetChTime() << " s, "
                      << "Position: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ") m, "
                      << "Velocity z: " << vel.z() << " m/s" << std::endl;
        }

        // Render and save snapshots
        if (enable_vis && vis && t >= render_frame / output_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            // Save snapshots at output FPS
            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << t << std::endl;
                std::ostringstream filename;
                filename << out_dir << "snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }
            render_frame++;
        }

        fsi.DoStepDynamics(step_size);
        t += step_size;
    }

    csv_output.close();
    if (snapshots && enable_vis) {
        std::cout << "Snapshots saved to: " << out_dir << "snapshots/" << std::endl;
    }
    std::cout << "CSV data saved to: " << csv_file << std::endl;

    return 0;
}
