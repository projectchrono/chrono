// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Huzaifa Mustafa Unjhawala
// =============================================================================
// Hydrostatic Test for constitutive model validation
// This demo tests the constitutive model under hydrostatic conditions
// without any penetrating objects
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono_fsi/sph/ChFsiSystemSPH.h"
#include "chrono_fsi/sph/ChFsiFluidSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

// -----------------------------------------------------------------------------
// Material properties
double nu_poisson = 0.3;

// Container material
struct solid_material {
    double youngs_modulus = 193e9;
    double friction_coefficient = 0.7;
    double density = 7.8e3;
    double restitution = 0.05;
    double adhesion = 0;
};

// -----------------------------------------------------------------------------

struct SimParams {
    // Simulation parameters
    int ps_freq;
    double initial_spacing;
    double d0_multiplier;
    double time_step;
    std::string boundary_type;
    std::string viscosity_type;
    std::string kernel_type;

    // Physics parameters
    double artificial_viscosity;
    double container_height;
    double test_duration;

    // Output/rendering parameters
    bool verbose;
    bool output;
    double output_fps;
    bool snapshots;
    bool render;
    double render_fps;
    bool write_marker_files;

    // Material properties
    double mu_s;
    double mu_2;
    double cohesion;
    double density;
    double y_modulus;
    std::string integration_scheme;
    std::string rheology_model_crm;
    double pre_pressure_scale;
};

void SimulateMaterial(int i, const SimParams& params);

// Function to handle CLI arguments
bool GetProblemSpecs(int argc, char** argv, SimParams& params) {
    ChCLI cli(argv[0], "FSI Hydrostatic Test Demo");

    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(params.ps_freq));
    cli.AddOption<double>("Simulation", "initial_spacing", "Initial spacing", std::to_string(params.initial_spacing));
    cli.AddOption<double>("Simulation", "d0_multiplier", "D0 multiplier", std::to_string(params.d0_multiplier));
    cli.AddOption<std::string>("Simulation", "boundary_type", "Boundary condition type (holmes/adami)",
                               params.boundary_type);
    cli.AddOption<std::string>("Simulation", "viscosity_type",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", params.viscosity_type);
    cli.AddOption<std::string>("Simulation", "kernel_type", "Kernel type (cubic/wendland)", params.kernel_type);
    cli.AddOption<double>("Simulation", "time_step", "Time step", std::to_string(params.time_step));

    cli.AddOption<double>("Physics", "artificial_viscosity", "Artificial viscosity",
                          std::to_string(params.artificial_viscosity));
    cli.AddOption<double>("Physics", "container_height", "Container height", std::to_string(params.container_height));
    cli.AddOption<double>("Physics", "test_duration", "Test duration", std::to_string(params.test_duration));
    cli.AddOption<double>("Physics", "mu_s", "Friction coefficient", std::to_string(params.mu_s));
    cli.AddOption<double>("Physics", "mu_2", "Friction coefficient", std::to_string(params.mu_2));
    cli.AddOption<double>("Physics", "cohesion", "Cohesion", std::to_string(params.cohesion));
    cli.AddOption<double>("Physics", "density", "Density", std::to_string(params.density));
    cli.AddOption<double>("Physics", "y_modulus", "Young's modulus", std::to_string(params.y_modulus));
    cli.AddOption<std::string>("Physics", "integration_scheme", "Integration scheme (euler/rk2)",
                               params.integration_scheme);
    cli.AddOption<std::string>("Physics", "rheology_model_crm", "Rheology model (MU_OF_I/MCC)",
                               params.rheology_model_crm);
    cli.AddOption<double>("Physics", "pre_pressure_scale", "Pre-pressure scale",
                          std::to_string(params.pre_pressure_scale));
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
    params.container_height = cli.GetAsType<double>("container_height");
    params.test_duration = cli.GetAsType<double>("test_duration");
    params.mu_s = cli.GetAsType<double>("mu_s");
    params.mu_2 = cli.GetAsType<double>("mu_2");
    params.cohesion = cli.GetAsType<double>("cohesion");
    params.density = cli.GetAsType<double>("density");
    params.y_modulus = cli.GetAsType<double>("y_modulus");
    params.integration_scheme = cli.GetAsType<std::string>("integration_scheme");
    params.rheology_model_crm = cli.GetAsType<std::string>("rheology_model_crm");
    params.pre_pressure_scale = cli.GetAsType<double>("pre_pressure_scale");
    return true;
}

int main(int argc, char* argv[]) {
    SimParams params = {/*ps_freq*/ 1,
                        /*initial_spacing*/ 0.001,
                        /*d0_multiplier*/ 1.3,
                        /*time_step*/ 2e-5,
                        /*boundary_type*/ "adami",
                        /*viscosity_type*/ "artificial_bilateral",
                        /*kernel_type*/ "wendland",
                        /*artificial_viscosity*/ 0.2,
                        /*container_height*/ 0.24,
                        /*test_duration*/ 4.0,  // 4 seconds test duration
                        /*verbose*/ true,
                        /*output*/ true,
                        /*output_fps*/ 20,
                        /*snapshots*/ true,
                        /*render*/ true,
                        /*render_fps*/ 200,
                        /*write_marker_files*/ true,
                        /*mu_s*/ 0.67,
                        /*mu_2*/ 0.67,
                        /*cohesion*/ 0,
                        /*density*/ 1600,
                        /*y_modulus*/ 1e6,
                        /*integration_scheme*/ "rk2",
                        /*rheology_model_crm*/ "MU_OF_I",
                        /*pre_pressure_scale*/ 5000};

    if (!GetProblemSpecs(argc, argv, params)) {
        return 1;
    }

    std::cout << "Problem Specs:" << std::endl;
    std::cout << "ps_freq: " << params.ps_freq << std::endl;
    std::cout << "initial_spacing: " << params.initial_spacing << std::endl;
    std::cout << "d0_multiplier: " << params.d0_multiplier << std::endl;
    std::cout << "time_step: " << params.time_step << std::endl;
    std::cout << "boundary_type: " << params.boundary_type << std::endl;
    std::cout << "viscosity_type: " << params.viscosity_type << std::endl;
    std::cout << "kernel_type: " << params.kernel_type << std::endl;
    std::cout << "artificial_viscosity: " << params.artificial_viscosity << std::endl;
    std::cout << "container_height: " << params.container_height << std::endl;
    std::cout << "test_duration: " << params.test_duration << std::endl;
    std::cout << "mu_s: " << params.mu_s << std::endl;
    std::cout << "mu_2: " << params.mu_2 << std::endl;
    std::cout << "cohesion: " << params.cohesion << std::endl;
    std::cout << "density: " << params.density << std::endl;
    std::cout << "y_modulus: " << params.y_modulus << std::endl;
    std::cout << "integration_scheme: " << params.integration_scheme << std::endl;
    std::cout << "rheology_model_crm: " << params.rheology_model_crm << std::endl;
    std::cout << "pre_pressure_scale: " << params.pre_pressure_scale << std::endl;

    int num_materials = 1;
    for (int i = 0; i < num_materials; i++) {
        SimulateMaterial(i, params);
    }
}

void SimulateMaterial(int i, const SimParams& params) {
    double t_end = params.test_duration;
    std::cout << "Test duration: " << t_end << " seconds" << std::endl;

    double container_diameter = 0.05;                   // container diameter (m)
    double container_height = params.container_height;  // configurable via CLI

    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFsiFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    // Set gravitational acceleration
    const ChVector3d gravity(0, 0, -9.81);
    sysSPH.SetGravitationalAcceleration(gravity);
    sysMBS.SetGravitationalAcceleration(gravity);

    sysFSI.SetStepSizeCFD(params.time_step);
    sysFSI.SetStepsizeMBD(params.time_step);

    sysSPH.SetVerbose(true);

    // -------------------------------------------------------------------------
    // Material and SPH parameters
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    ChFsiFluidSystemSPH::SPHParameters sph_params;

    mat_props.density = params.density;
    mat_props.Young_modulus = params.y_modulus;
    mat_props.Poisson_ratio = nu_poisson;
    if (params.rheology_model_crm == "MU_OF_I") {
        mat_props.rheology_model = RheologyCRM::MU_OF_I;
        mat_props.mu_I0 = 0.04;
        mat_props.mu_fric_s = params.mu_s;
        mat_props.mu_fric_2 = params.mu_2;
        mat_props.average_diam = 0.0002;
        mat_props.cohesion_coeff = params.cohesion;
    } else {
        mat_props.rheology_model = RheologyCRM::MCC;
        double angle_mus = std::atan(params.mu_s);
        mat_props.mcc_M = 1.3;
        mat_props.mcc_kappa = 0.01;
        mat_props.mcc_lambda = 0.1;
    }

    sysSPH.SetElasticSPH(mat_props);
    if (params.integration_scheme == "euler") {
        sph_params.integration_scheme = IntegrationScheme::EULER;
    } else if (params.integration_scheme == "rk2") {
        sph_params.integration_scheme = IntegrationScheme::RK2;
    }
    sph_params.initial_spacing = params.initial_spacing;
    sph_params.num_bce_layers = 5;
    sph_params.d0_multiplier = params.d0_multiplier;
    sph_params.artificial_viscosity = params.artificial_viscosity;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 2.0;
    sph_params.num_proximity_search_steps = params.ps_freq;

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
    // -------------------------------------------------------------------------

    // ==============================
    // Create container and granular material
    // ==============================
    // Create a container
    double clearance = 0.2 * container_height;
    double bxDim = container_diameter;
    double byDim = container_diameter;
    double fzDim = container_height;
    double bzDim = fzDim + clearance;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 * 1.2, -byDim / 2 * 1.2, -bzDim * 1.2);
    ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, bzDim * 1.2);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);

    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Create granular material
    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(params.initial_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - params.initial_spacing, byDim / 2 - params.initial_spacing,
                          fzDim / 2 - params.initial_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double depth_cm = (-p.z() + fzDim) * 100;
        double b = 12.2;
        double c = 18;
        double fzDim_cm = fzDim * 100;
        double g = (depth_cm + b) / (depth_cm + c);
        double gbar = 1.0 - ((c - b) / fzDim_cm) * std::log((c + fzDim_cm) / c);
        double density_mean_target = params.density;
        double rho_ini = density_mean_target * g / gbar;
        double pre_ini = rho_ini * gz * (-p.z() + fzDim);
        double preconsidation_pressure = pre_ini + params.pre_pressure_scale;
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0),
                              ChVector3d(-pre_ini, -pre_ini, -pre_ini), ChVector3d(0, 0, 0), preconsidation_pressure);
    }

    solid_material solid_mat;
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    auto vis_material = chrono_types::make_shared<ChVisualMaterial>();
    cmaterial->SetYoungModulus(solid_mat.youngs_modulus);
    cmaterial->SetFriction(solid_mat.friction_coefficient);
    cmaterial->SetRestitution(solid_mat.restitution);
    cmaterial->SetAdhesion(solid_mat.adhesion);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);
    // Add BCE particles attached on the walls into FSI system (new API)
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), ChVector3i(2, 2, -1));
    sysFSI.AddFsiBoundary(box_bce, ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT));

    sysSPH.SetOutputLevel(OutputLevel::CRM_FULL);
    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (params.output || params.snapshots) {
        // Base output directory for hydrostatic test
        std::string base_dir;
        const std::string heightCmStr = [&]() {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << (params.container_height * 100.0) << "cm";
            return oss.str();
        }();

        // Helper lambda to convert a value to a string using an ostringstream.
        auto toString = [](auto value) -> std::string {
            std::ostringstream oss;
            oss << value;
            return oss.str();
        };

        // Convert new parameters to strings
        const std::string rheologyModelStr = params.rheology_model_crm;
        const std::string prePressureScaleStr = toString(params.pre_pressure_scale);
        base_dir = GetChronoOutputPath() + "FSI_Hydrostatic_" + heightCmStr + "_" + rheologyModelStr + "_" +
                   prePressureScaleStr + "/";
        if (!filesystem::create_directory(filesystem::path(base_dir))) {
            std::cerr << "Error creating directory " << base_dir << std::endl;
            return;
        }

        // Convert array values to strings.
        const std::string youngsModulusStr = toString(params.y_modulus);
        const std::string densityStr = toString(params.density);
        const std::string muSStr = toString(params.mu_s);
        const std::string mu2Str = toString(params.mu_2);
        const std::string cohesionStr = toString(params.cohesion);

        // Build the vector of subdirectory names.
        std::vector<std::string> subdirs = {"youngsModulus_" + youngsModulusStr,
                                            "density_" + densityStr,
                                            "mu_s_" + muSStr,
                                            "mu_2_" + mu2Str,
                                            "cohesion_" + cohesionStr,
                                            "boundaryType_" + params.boundary_type,
                                            "viscosityType_" + params.viscosity_type,
                                            "kernelType_" + params.kernel_type};

        for (const auto& subdir : subdirs) {
            base_dir += subdir + "/";
            if (!filesystem::create_directory(filesystem::path(base_dir))) {
                std::cerr << "Error creating directory " << base_dir << std::endl;
                return;
            }
        }

        // Add flat structure
        std::stringstream ss;
        ss << "ps_" << params.ps_freq;
        ss << "_s_" << params.initial_spacing;
        ss << "_d0_" << params.d0_multiplier;
        ss << "_t_" << params.time_step;
        ss << "_av_" << params.artificial_viscosity;
        out_dir = base_dir + ss.str();

        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return;
        }

        if (params.output) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
                std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
                return;
            }
        }

        if (params.snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return;
            }
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;
#ifdef CHRONO_VSG
    auto col_callback = chrono_types::make_shared<ParticlePressureColorCallback>(0, 30000, false);
    if (params.render) {
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(true);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("FSI Hydrostatic Test");
        visVSG->SetWindowSize(1280, 720);
        visVSG->AddCamera(ChVector3d(0, -2 * container_height, 0.75 * container_height),
                          ChVector3d(0, 0, 0.55 * container_height));
        visVSG->SetLightIntensity(0.9);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();

    // Calculate hydrostatic pressure statistics
    double avg_pressure = 0.0;
    double avg_density = 0.0;
    double max_pressure = 0.0;
    double min_pressure = std::numeric_limits<double>::max();

    avg_pressure = params.density * 9.81 * container_height / 2.0;  // Hydrostatic pressure at mid-height
    avg_density = params.density;
    max_pressure = params.density * 9.81 * container_height;  // Pressure at bottom
    min_pressure = 0.0;                                       // Pressure at top

    std::cout << "Avg Pressure: " << avg_pressure << std::endl;
    std::cout << "Avg Density: " << avg_density << std::endl;
    std::cout << "Max Pressure: " << max_pressure << std::endl;
    std::cout << "Min Pressure: " << min_pressure << std::endl;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (params.output && time >= out_frame / params.output_fps) {
            if (params.write_marker_files) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
            }
            std::cout << "Time: " << time << " seconds" << std::endl;
            out_frame++;
        }

#ifdef CHRONO_VSG
        if (params.render && time >= render_frame / params.render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (params.snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif

        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;
}
