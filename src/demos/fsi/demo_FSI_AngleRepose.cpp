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
// Author: Wei Hu, Huzaifa Mustafa Unjhawala
// =============================================================================

// General Includes
#include <cassert>
#include <cstdlib>
#include <ctime>
#include <iomanip>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChSphVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono/utils/ChUtilsSamplers.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::fsi::sph;

// Data from Wei Paper -
// https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff

double bulk_density = 1500;
double mu_s = 0.3819;
double granular_particle_diameter = 0.002;
double youngs_modulus = 2e6;

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& snapshots,
                     int& ps_freq,
                     double& cylinder_radius,
                     double& cylinder_height,
                     double& init_spacing,
                     bool& render) {
    ChCLI cli(argv[0], "FSI Angle of Repose Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<bool>("Simulation", "verbose", "Verbose output", std::to_string(verbose));
    cli.AddOption<bool>("Output", "output", "Enable output", std::to_string(output));
    cli.AddOption<double>("Output", "output_fps", "Output FPS", std::to_string(output_fps));
    cli.AddOption<bool>("Output", "snapshots", "Enable snapshots", std::to_string(snapshots));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Geometry", "cylinder_radius", "Cylinder radius", std::to_string(cylinder_radius));
    cli.AddOption<double>("Geometry", "cylinder_height", "Cylinder height", std::to_string(cylinder_height));
    cli.AddOption<double>("Simulation", "init_spacing", "Initial particle spacing", std::to_string(init_spacing));

    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    verbose = cli.GetAsType<bool>("verbose");
    output = cli.GetAsType<bool>("output");
    output_fps = cli.GetAsType<double>("output_fps");
    snapshots = cli.GetAsType<bool>("snapshots");
    ps_freq = cli.GetAsType<int>("ps_freq");
    cylinder_radius = cli.GetAsType<double>("cylinder_radius");
    cylinder_height = cli.GetAsType<double>("cylinder_height");
    init_spacing = cli.GetAsType<double>("init_spacing");
    render = !cli.GetAsType<bool>("no_vis");
    return true;
}

int main(int argc, char* argv[]) {
    // Default values
    double t_end = 10.0;
    bool verbose = false;
    bool output = true;
    double output_fps = 20;
    bool snapshots = false;
    int ps_freq = 1;
    double cylinder_radius = 0.5;
    double cylinder_height = 1.0;
    double init_spacing = 0.01;
    bool render = true;
    double render_fps = 100;
    double step_size = 1e-4;
    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, snapshots, ps_freq, cylinder_radius,
                         cylinder_height, init_spacing, render)) {
        return 1;
    }

    ChSystemSMC sysMBS;
    ChFsiFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);
    sysFSI.SetVerbose(verbose);
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    // Set soil propertiees
    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = bulk_density;
    mat_props.Young_modulus = youngs_modulus;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.03;
    mat_props.mu_fric_s = mu_s;
    mat_props.mu_fric_2 = mu_s;
    mat_props.average_diam = granular_particle_diameter;
    sysSPH.SetElasticSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = init_spacing;
    sph_params.d0_multiplier = 1.2;
    sph_params.artificial_viscosity = 0.5;
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.25;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 0.8;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.use_variable_time_step = false;
    sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    sph_params.boundary_method = BoundaryMethod::ADAMI;
    sysSPH.SetSPHParameters(sph_params);

    // Dimension of the space domain
    double bxDim = 10 * cylinder_radius;
    double byDim = 10 * cylinder_radius;
    double bzDim = 1.5 * cylinder_height;  // Higher than the cylinder to allow forparticle settling

    // Set the periodic boundary condition
    auto initSpace0 = sysSPH.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 - 3 * initSpace0 / 2.0, -byDim / 2 - 3 * initSpace0 / 2.0,
                    -1.0 * bzDim - 3 * initSpace0);
    ChVector3d cMax(bxDim / 2 + 3 * initSpace0 / 2.0, byDim / 2 + 3 * initSpace0 / 2.0, 2.0 * bzDim + 3 * initSpace0);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);
    sysSPH.SetOutputLevel(OutputLevel::CRM_FULL);

    // Create SPH particle locations using a sampler
    chrono::utils::ChGridSampler<> sampler(init_spacing);
    ChVector3d cylinderCenter(0.0, 0.0, -bzDim / 2 + cylinder_height / 2 + 2 * init_spacing);
    std::vector<ChVector3d> points = sampler.SampleCylinderZ(cylinderCenter, cylinder_radius, cylinder_height / 2);

    // Add fluid particles
    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-(p.z() + cylinder_height / 2) + cylinder_height);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0));
    }

    // Add a box
    // Set common material Properties
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.389f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                         //
                                   ChFrame<>(ChVector3d(0, 0, 0), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,   //
                                   ChVector3i(0, 0, -1),                   //
                                   false);
    box->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), {0, 0, -1});
    sysFSI.AddFsiBody(box, box_bce, ChFrame<>(ChVector3d(0, 0, 0), QUNIT), false);

    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (output || snapshots) {
        out_dir = GetChronoOutputPath() + "FSI_Angle_Repose/";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        std::stringstream ss;
        ss << "ps" << ps_freq;
        ss << "_r" << cylinder_radius;
        ss << "_h" << cylinder_height;
        ss << "_s" << init_spacing;
        out_dir = out_dir + ss.str();
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        if (output) {
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

        if (snapshots) {
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

    // Create a run-time visualizer
    std::shared_ptr<ChVisualSystem> vis;

#ifdef CHRONO_VSG
    if (render) {
        // FSI plugin
        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Angle of Repose");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -3 * byDim, bzDim), ChVector3d(0, 0, 0));
        visVSG->SetLightIntensity(0.9f);
        visVSG->SetLightDirection(-CH_PI_2, CH_PI / 6);

        visVSG->Initialize();
        vis = visVSG;
    }
#else
    render = false;
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;
    double dT = sysFSI.GetStepSizeCFD();
    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            out_frame++;
        }

        // Render SPH particles
#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif

        if (sim_frame % 1000 == 0) {
            std::cout << "step: " << sim_frame << "\ttime: " << time << "\tRTF: " << sysFSI.GetRtf() << std::endl;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics(dT);

        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
