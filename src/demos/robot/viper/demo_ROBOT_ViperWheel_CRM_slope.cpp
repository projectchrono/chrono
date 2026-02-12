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
// Author: Wei Hu, Radu Serban, Huzaifa Mustafa Unjhawala
// =============================================================================

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <fstream>

#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/ChConfig.h"
#include "chrono/core/ChTimer.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChMassProperties.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/utils/ChUtilsSamplers.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "viper_wheel.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Physical properties of terrain particles
double density = 1760.0;

// Dimension of the terrain container
double smalldis = 1.0e-9;
double bxDim = 5.0 + smalldis;
double byDim = 0.8 + smalldis;
double bzDim = 0.2 + smalldis;

// Variables from DEM sim
double safe_x = 1.0;
double z_adv_targ = 0.2;

// Size of the wheel
double wheel_radius = 0.225;
double wheel_slip = 0.0;
double wheel_width = 0.200;
double grouser_wide = 0.005;
int grouser_num = 24;
std::string wheel_obj = "robot/viper/obj/nasa_viper_wheel.obj";

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;
    double artificial_viscosity;
    std::string integration_scheme;
    double total_time;
    bool use_variable_time_step;

    // Physical parameters of experiment
    double total_mass;
    double slope_angle;
    double wheel_AngVel;
    double gravity_G;
    double grouser_height;
    int sim_number;
    bool snapshots;  // Whether to save snapshot image files

    // Output parameters
    bool output;
    double out_fps;
    bool write_marker_files;
    double print_fps;
    bool render;
    double render_fps;
};

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChFsiSystemSPH& sysFSI,
                      ChVector3d& wheel_IniPos,
                      ChVector3d& wheel_IniVel,
                      double wheel_AngVel,
                      bool render_wheel,
                      double grouser_height,
                      double kernelLength,
                      double total_mass,
                      double iniSpacing) {
    ChFsiFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();
    ChSystem& sysMBS = sysFSI.GetMultibodySystem();

    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e9);
    cmaterial->SetFriction(1.f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetPos(ChVector3d(0., 0., 0.));
    ground->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ground->SetFixed(true);
    sysMBS.AddBody(ground);
    double multiplier = 1.0;
    chrono::utils::AddBoxContainer(ground, cmaterial,                                               //
                                   ChFrame<>(ChVector3d(0., 0., 0.), QUNIT),                        //
                                   ChVector3d(multiplier * bxDim, multiplier * byDim, bzDim), 0.1,  //
                                   ChVector3i(2, 2, 2),                                             //
                                   false);
    ground->EnableCollision(true);

    // Add BCE particles attached on the walls into FSI system
    auto ground_bce =
        sysSPH.CreatePointsBoxContainer(ChVector3d(multiplier * bxDim, multiplier * byDim, bzDim), {2, 0, -1});
    sysFSI.AddFsiBoundary(ground_bce, ChFrame<>(ChVector3d(0., 0., 0.), QUNIT));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0),
                       ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);         // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = 1500.;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);
    std::cout << "This is unused but... " << std::endl;
    std::cout << "Wheel Mass: " << mmass << std::endl;
    std::cout << "Wheel Density: " << mdensity << std::endl;
    std::cout << "Inertia Matrix: " << std::endl << minertia << std::endl;

    // Set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> wheel_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF
    // reference. Make the COG frame a principal frame.
    wheel->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(mdensity * principal_I);
    wheel->SetPosDt(wheel_IniVel);
    wheel->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // wheel material
    auto cmaterial2 = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial2->SetYoungModulus(1e9);
    cmaterial2->SetFriction(0.9f);
    cmaterial2->SetRestitution(0.4f);
    cmaterial2->SetAdhesion(0);
    // Set the absolute position of the body:
    wheel->SetFrameRefToAbs(ChFrame<>(ChVector3d(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    wheel->SetFixed(false);
    auto wheel_shape =
        chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial2, trimesh, false, false, 0.005);
    wheel->AddCollisionShape(wheel_shape);
    wheel->EnableCollision(false);
    if (render_wheel) {
        auto wheel_visual_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
        wheel_visual_shape->SetMesh(trimesh);
        wheel->AddVisualShape(wheel_visual_shape, ChFrame<>());
    }

    sysMBS.AddBody(wheel);

    // Create wheel FSI body
    double inner_radius = wheel_radius;
    double outer_radius = wheel_radius + grouser_height;
    auto bce = CreateWheelBCE(inner_radius, wheel_width - iniSpacing, grouser_height, grouser_wide, grouser_num,
                              iniSpacing, false);
    ChQuaternion<> wheel_Rot_bce = Q_ROTATE_Z_TO_Y;
    sysFSI.AddFsiBody(wheel, bce, ChFrame<>(ChVector3d(0, 0, 0), ChQuaternion<>(wheel_Rot_bce)), false);

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 3.0);
    chassis->SetPos(wheel->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    // Add geometry of the axle.
    // chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5,
    //                                  ChVector3d(0, 0, 0));
    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational
    // joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "FSI Viper Single Wheel Demo");

    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(params.ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(params.initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(params.d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)",
                               params.boundary_type);
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", params.viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", params.kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));
    cli.AddOption<double>("Simulation", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(params.artificial_viscosity));
    cli.AddOption<std::string>("Physics", "integration_scheme", "Integration scheme (euler/rk2)",
                               params.integration_scheme);
    cli.AddOption<double>("Simulation", "total_time", "Total time", std::to_string(params.total_time));
    std::string use_variable_time_step_str = params.use_variable_time_step ? "true" : "false";
    cli.AddOption<std::string>("Simulation", "use_variable_time_step", "Use variable time step",
                               use_variable_time_step_str);

    cli.AddOption<double>("Physics", "total_mass", "Total mass", std::to_string(params.total_mass));
    cli.AddOption<double>("Physics", "slope_angle", "Slope angle", std::to_string(params.slope_angle));
    cli.AddOption<double>("Physics", "wheel_AngVel", "Wheel angular velocity", std::to_string(params.wheel_AngVel));
    cli.AddOption<double>("Physics", "gravity_G", "Gravity", std::to_string(params.gravity_G));
    cli.AddOption<double>("Physics", "grouser_height", "Grouser height", std::to_string(params.grouser_height));
    cli.AddOption<int>("Simulation", "sim_number", "Simulation number", std::to_string(params.sim_number));
    std::string snapshots_str = params.snapshots ? "true" : "false";
    cli.AddOption<std::string>("Visualization", "snapshots", "Enable writing snapshot image files",
                               params.snapshots ? "true" : "false");
    std::string output_str = params.output ? "true" : "false";
    cli.AddOption<std::string>("Output", "output", "Enable output", output_str);
    cli.AddOption<double>("Output", "out_fps", "Output frequency", std::to_string(params.out_fps));
    std::string write_marker_files_str = params.write_marker_files ? "true" : "false";
    cli.AddOption<std::string>("Output", "write_marker_files", "Enable writing marker files", write_marker_files_str);
    cli.AddOption<double>("Output", "print_fps", "Print frequency", std::to_string(params.print_fps));
    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency", std::to_string(params.render_fps));

    if (!cli.Parse(argc, argv))
        return false;

    params.ps_freq = cli.GetAsType<int>("ps_freq");
    params.initial_spacing = cli.GetAsType<double>("initial_spacing");
    params.d0_multiplier = cli.GetAsType<double>("d0_multiplier");
    params.time_step = cli.GetAsType<double>("time_step");
    params.boundary_type = cli.GetAsType<std::string>("boundary_type");
    params.viscosity_type = cli.GetAsType<std::string>("viscosity_type");
    params.kernel_type = cli.GetAsType<std::string>("kernel_type");
    params.artificial_viscosity = cli.GetAsType<double>("artificial_viscosity");
    params.integration_scheme = cli.GetAsType<std::string>("integration_scheme");
    params.total_time = cli.GetAsType<double>("total_time");
    params.use_variable_time_step = parse_bool(cli.GetAsType<std::string>("use_variable_time_step"));
    params.total_mass = cli.GetAsType<double>("total_mass");
    params.slope_angle = cli.GetAsType<double>("slope_angle");
    params.wheel_AngVel = cli.GetAsType<double>("wheel_AngVel");
    params.gravity_G = cli.GetAsType<double>("gravity_G");
    params.grouser_height = cli.GetAsType<double>("grouser_height");
    params.sim_number = cli.GetAsType<int>("sim_number");
    params.snapshots = parse_bool(cli.GetAsType<std::string>("snapshots"));
    params.output = parse_bool(cli.GetAsType<std::string>("output"));
    params.out_fps = cli.GetAsType<double>("out_fps");
    params.write_marker_files = parse_bool(cli.GetAsType<std::string>("write_marker_files"));
    params.print_fps = cli.GetAsType<double>("print_fps");
    params.render = !cli.GetAsType<bool>("no_vis");
    params.render_fps = cli.GetAsType<double>("render_fps");

    return true;
}

int main(int argc, char* argv[]) {
    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;

#ifdef NDEBUG
    std::cout << "Disable cuda error check in RELEASE mode" << std::endl;
    sysSPH.EnableCudaErrorCheck(false);
#endif
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Set default simulation parameters
    SimParams params = {/*ps_freq*/ 1,
                        /*initial_spacing*/ 0.01,
                        /*d0_multiplier*/ 1.2,
                        /*time_step*/ 2e-4,
                        /*boundary_type*/ "adami",
                        /*viscosity_type*/ "artificial_bilateral",
                        /*kernel_type*/ "cubic",
                        /*artificial_viscosity*/ 0.05,
                        /*integration_scheme*/ "rk2",
                        /*total_time*/ 10.0,
                        /*use_variable_time_step*/ false,
                        /*total_mass*/ 100,
                        /*slope_angle*/ 0.0,
                        /*wheel_AngVel*/ 0.1,
                        /*gravity_G*/ 9.8,
                        /*grouser_height*/ 0.01,
                        /*sim_number*/ 0,
                        /*snapshots*/ false,
                        /*output*/ false,
                        /*out_fps*/ 100,
                        /*write_marker_files*/ false,
                        /*print_fps*/ 100,
                        /*render*/ true,
                        /*render_fps*/ 100};

    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    std::cout << "Problem Specs:" << std::endl;
    std::cout << "ps_freq: " << params.ps_freq << std::endl;
    std::cout << "initial_spacing: " << params.initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << params.d0_multiplier << std::endl;
    std::cout << "use_variable_time_step: " << (params.use_variable_time_step ? "true" : "false") << std::endl;
    if (params.use_variable_time_step) {
        std::cout << "time_step used for the first time step: " << params.time_step << std::endl;
    } else {
        std::cout << "time_step: " << params.time_step << std::endl;
    }
    std::cout << "boundary_type: " << params.boundary_type << std::endl;
    std::cout << "viscosity_type: " << params.viscosity_type << std::endl;
    std::cout << "kernel_type: " << params.kernel_type << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;
    std::cout << "integration_scheme: " << params.integration_scheme << std::endl;
    std::cout << "snapshots: " << (params.snapshots ? "true" : "false") << std::endl;

    sysFSI.SetVerbose(true);

    std::cout << "Total Mass: " << params.total_mass << std::endl;
    std::cout << "Slope Angle: " << params.slope_angle << std::endl;
    std::cout << "Wheel Angular Velocity: " << params.wheel_AngVel << std::endl;
    std::cout << "Gravity Magnitude: " << params.gravity_G << std::endl;
    std::cout << "Grouser Height: " << params.grouser_height << std::endl;
    params.slope_angle = params.slope_angle / 180.0 * CH_PI;

    // Create formatted output directory path with appropriate precision
    std::string out_dir = GetChronoOutputPath() + "VIPER_WHEEL_CRM_SLOPE";

    if (params.output) {
        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        std::stringstream ss;
        ss << std::fixed;
        ss << out_dir + "/ps_" << params.ps_freq;
        ss << "_s_" << std::setprecision(3) << params.initial_spacing;
        ss << "_d0_" << std::setprecision(1) << params.d0_multiplier;
        ss << "_av_" << std::setprecision(2) << params.artificial_viscosity << "/";
        out_dir = ss.str();

        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        std::string sim_number_str = std::to_string(params.sim_number);
        out_dir = out_dir + sim_number_str + "/";

        // Output the result to verify
        std::cout << "Output directory: " << out_dir << std::endl;

        // Create output directories
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
            std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
            std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
            return 1;
        }
    }

    // Create directory for snapshots if enabled
    if (params.snapshots) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "snapshots"))) {
            std::cerr << "Error creating directory " << out_dir + "snapshots" << std::endl;
            return 1;
        }
    }

    // Set the simulation stepsize
    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    // Meta-step (communication interval)
    double meta_time_step = 5 * params.time_step;

    // We simulate slope by just tilting the gravity vector
    double gravity_G = -params.gravity_G;
    ChVector3d gravity = ChVector3d(gravity_G * sin(params.slope_angle), 0, gravity_G * cos(params.slope_angle));
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Set Parameters
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    mat_props.density = density;
    mat_props.Young_modulus = 1e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.793;
    mat_props.mu_fric_2 = 0.793;
    mat_props.average_diam = 0.005;
    mat_props.cohesion_coeff = 0;

    sysSPH.SetElasticSPH(mat_props);

    if (params.integration_scheme == "euler") {
        sph_params.integration_scheme = IntegrationScheme::EULER;
    } else if (params.integration_scheme == "rk2") {
        sph_params.integration_scheme = IntegrationScheme::RK2;
    }

    sph_params.initial_spacing = params.initial_spacing;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 0.8;
    sph_params.num_proximity_search_steps = params.ps_freq;
    sph_params.use_variable_time_step = params.use_variable_time_step;

    if (params.kernel_type == "cubic") {
        sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    } else if (params.kernel_type == "wendland") {
        sph_params.kernel_type = KernelType::WENDLAND;
    }
    if (params.boundary_type == "holmes") {
        sph_params.boundary_method = BoundaryMethod::HOLMES;
    } else {
        sph_params.boundary_method = BoundaryMethod::ADAMI;
    }

    // Set viscosity type
    if (params.viscosity_type == "artificial_bilateral") {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    }

    sysSPH.SetSPHParameters(sph_params);

    double iniSpacing = params.initial_spacing;
    double kernelLength = params.initial_spacing * params.d0_multiplier;

    // Initial Position of wheel
    ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius * 2.0, 0.0, wheel_radius + 10 * iniSpacing);
    ChVector3d wheel_IniVel(0.0, 0.0, 0.0);

    // Set the computational domain limits
    ChVector3d cMin(-bxDim / 2 - 3 * iniSpacing, -byDim / 2, -bzDim - 30 * iniSpacing);
    ChVector3d cMax(bxDim / 2 + 3 * iniSpacing, byDim / 2, bzDim + 30 * iniSpacing);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_Y_PERIODIC);

    // Initialize the SPH particles
    chrono::utils::ChGridSampler<> sampler(iniSpacing);
    ChVector3d boxCenter(0.0, 0.0, 0.0);
    ChVector3d boxHalfDim(bxDim / 2 - iniSpacing, byDim / 2 - iniSpacing, bzDim / 2 - iniSpacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = (int)points.size();
    double gz = params.gravity_G;
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysSPH.GetDensity() * gz * (-points[i].z() + bzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(points[i], rho_ini, pre_ini, sysSPH.GetViscosity());
    }

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysFSI, wheel_IniPos, wheel_IniVel, params.wheel_AngVel, params.render, params.grouser_height,
                     kernelLength, params.total_mass, params.initial_spacing);

    sysSPH.SetActiveDomain(ChVector3d(0.6, 0.6, 0.8));
    sysSPH.SetActiveDomainDelay(1.0);
    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];
    auto reaction = actuator->GetReaction2();
    ChVector3d force = reaction.force;
    ChVector3d torque = motor->GetReaction1().torque;
    ChVector3d w_pos = wheel->GetPos();
    ChVector3d w_vel = wheel->GetPosDt();
    ChVector3d angvel = wheel->GetAngVelLocal();

    // Save wheel mesh
    ChTriangleMeshConnected wheel_mesh;
    wheel_mesh.LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (params.output) {
        if (params.use_variable_time_step) {
            myFile.open(out_dir + "/results_variable_time_step.txt", std::ios::trunc);
            myDBP_Torque.open(out_dir + "/DBP_Torque_variable_time_step.txt", std::ios::trunc);
        } else {
            myFile.open(out_dir + "/results_fixed_time_step.txt", std::ios::trunc);
            myDBP_Torque.open(out_dir + "/DBP_Torque_fixed_time_step.txt", std::ios::trunc);
        }
    }

    myFile << "time\t"
           << "x\t"
           << "y\t"
           << "z\t"
           << "vx\t"
           << "vy\t"
           << "vz\t"
           << "ax\t"
           << "ay\t"
           << "az\t"
           << "fx\t"
           << "fy\t"
           << "fz\t"
           << "tx\t"
           << "ty\t"
           << "tz\n";

    myDBP_Torque << "time\t"
                 << "fx\t"
                 << "tz\n";

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    // Set up real-time visualization of the FSI system
    if (params.render) {
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(false);
        visFSI->EnableRigidBodyMarkers(false);

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Chrono::CRM single wheel test");
        visVSG->SetWindowSize(1280, 960);
        visVSG->AddCamera(ChVector3d(-bxDim / 2. + 1, -5 * byDim, 5 * bzDim), ChVector3d(-bxDim / 2. + 1, 0., 0));
        visVSG->SetLightIntensity(0.9);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif

    double time = 0.0;
    int render_frame = 0;
    int output_frame = 0;
    int print_frame = 0;
    // Some things that even DEM domes
    double x1 = wheel->GetPos().x();
    double z1 = x1 * std::sin(params.slope_angle);
    double x2 = x1;
    double z_adv = 0;
    int counter = 0;

    ChTimer timer;
    timer.start();
    while (time < params.total_time) {
        double adv = x2 - x1;
        if (time >= print_frame / params.print_fps) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            // Compute slip
            // double linear_velocity = w_vel.x();
            double angular_velocity = angvel.z();
            // double slip = 1 - (std::abs(linear_velocity) / (std::abs(angular_velocity) * (wheel_radius +
            // grouser_height))); Compute slip like DEM
            double slip = 1 - (adv / (std::abs(angular_velocity) * (wheel_radius + params.grouser_height) * time));
            // Print slip
            std::cout << "Slip: " << slip << std::endl;

            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
            print_frame++;
        }

        if (z_adv >= z_adv_targ) {
            break;
        }

        if (params.output && time >= output_frame / params.out_fps) {
            std::cout << "-------- Output" << std::endl;
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";
            if (params.write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
                WriteWheelVTK(filename, wheel_mesh, wheel->GetFrameRefToAbs());
            }
            output_frame++;
        }

        // Render SPH particles
#ifdef CHRONO_VSG
        if (params.render && time >= render_frame / params.render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            // Save snapshots if enabled
            if (params.snapshots) {
                std::cout << "-------- Snapshot" << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
                render_frame++;
            }
            render_frame++;
        }
#endif

        // Get the infomation of the wheel
        reaction = actuator->GetReaction2();
        force = reaction.force;
        torque = motor->GetReaction1().torque;
        w_pos = wheel->GetPos();
        w_vel = wheel->GetPosDt();
        angvel = wheel->GetAngVelLocal();

        x2 = w_pos.x();
        z_adv = x2 * std::sin(params.slope_angle) - z1;
        if (x2 > safe_x) {
            break;
        }

        // Advance dynamics of the FSI system to next communication time
        sysFSI.DoStepDynamics(meta_time_step);

        time += meta_time_step;
    }
    timer.stop();
    if (params.output) {
        myFile.close();
        myDBP_Torque.close();
    }
    std::cout << "Runtime: " << timer() << " seconds\n" << std::endl;
    std::cout << "Simulation time: " << time << std::endl;
    std::cout << "Simulation finished" << std::endl;

    // Create RTF file
    std::ofstream rtf_file;
    if (params.use_variable_time_step) {
        rtf_file.open(out_dir + "/rtf_variable_time_step.txt", std::ios::trunc);
    } else {
        rtf_file.open(out_dir + "/rtf_fixed_time_step.txt", std::ios::trunc);
    }
    rtf_file << "time (s)"
             << "\t"
             << "wall clock time (s)"
             << "\t"
             << "RTF" << std::endl;
    rtf_file << time << "\t" << timer() << "\t" << timer() / time << std::endl;
    rtf_file.close();

    if (params.use_variable_time_step) {
        sysSPH.PrintStats();
        sysSPH.PrintTimeSteps(out_dir + "/time_steps.txt");
    }

    return 0;
}
