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
// Cratering validation problem involving spherical impactors with different
// densities falling from different heights (zero velocity) on CRM soil.
//
// Reference solution:
// https://www.sciencedirect.com/science/article/pii/S0045782521003534?ref=pdf_download&fr=RR-2&rr=8c4472d7d99222ff
//
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <iomanip>
#include <fstream>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/assets/ChVisualSystem.h"
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

const double sphere_radius = 0.0125;

// -----------------------------------------------------------------------------

#ifdef CHRONO_VSG
class MarkerPositionVisibilityCallback : public ChSphVisualizationVSG::MarkerVisibilityCallback {
  public:
    MarkerPositionVisibilityCallback() {}
    virtual bool get(unsigned int n) const override { return pos[n].y > 0; }
};
#endif

// -----------------------------------------------------------------------------

// Function to handle CLI arguments
bool GetProblemSpecs(int argc,
                     char** argv,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& snapshots,
                     int& ps_freq,
                     double& sphere_density,
                     double& Hdrop,
                     bool& render,
                     std::string& boundary_method,
                     std::string& viscosity_method) {
    ChCLI cli(argv[0], "FSI Cratering Demo");

    cli.AddOption<double>("Simulation", "t_end", "End time", std::to_string(t_end));
    cli.AddOption<bool>("Simulation", "verbose", "Verbose output", std::to_string(verbose));
    cli.AddOption<bool>("Output", "output", "Enable output", std::to_string(output));
    cli.AddOption<double>("Output", "output_fps", "Output FPS", std::to_string(output_fps));
    cli.AddOption<bool>("Output", "snapshots", "Enable snapshots", std::to_string(snapshots));
    cli.AddOption<int>("Simulation", "ps_freq", "Proximity search frequency", std::to_string(ps_freq));
    cli.AddOption<double>("Geometry", "sphere_density", "Sphere density", std::to_string(sphere_density));
    cli.AddOption<double>("Geometry", "Hdrop", "Drop height", std::to_string(Hdrop));
    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");

    cli.AddOption<std::string>("Physics", "boundary_method", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_method",
                               "Viscosity type (artificial_unilateral/artificial_bilateral)", "artificial_unilateral");

    if (!cli.Parse(argc, argv))
        return false;

    t_end = cli.GetAsType<double>("t_end");
    verbose = cli.GetAsType<bool>("verbose");
    output = cli.GetAsType<bool>("output");
    output_fps = cli.GetAsType<double>("output_fps");
    snapshots = cli.GetAsType<bool>("snapshots");
    ps_freq = cli.GetAsType<int>("ps_freq");
    sphere_density = cli.GetAsType<double>("sphere_density");
    Hdrop = cli.GetAsType<double>("Hdrop");
    render = !cli.GetAsType<bool>("no_vis");

    boundary_method = cli.GetAsType<std::string>("boundary_method");
    viscosity_method = cli.GetAsType<std::string>("viscosity_method");

    return true;
}

int main(int argc, char* argv[]) {
    // Default values
    double t_end = 2.0;
    bool verbose = true;
    bool output = true;
    double output_fps = 20;
    bool snapshots = false;
    int ps_freq = 1;
    double sphere_density = 700;
    double Hdrop = 0.5;
    bool render = true;
    double render_fps = 400;
    double step_size = 5e-5;
    double init_spacing = 0.0025;
    std::string boundary_method = "adami";
    std::string viscosity_method = "artificial_unilateral";

    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, snapshots, ps_freq, sphere_density, Hdrop,
                         render, boundary_method, viscosity_method)) {
        return 1;
    }

    // Create a physics system
    ChSystemSMC sysMBS;
    // Create a fluid system
    ChFsiFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(&sysMBS, &sysSPH);
    sysFSI.SetStepSizeCFD(step_size);
    sysFSI.SetStepsizeMBD(step_size);
    double g = 9.81;
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, -g));
    sysMBS.SetGravitationalAcceleration(sysSPH.GetGravitationalAcceleration());

    sysFSI.SetVerbose(verbose);

    ChFsiFluidSystemSPH::ElasticMaterialProperties mat_props;
    mat_props.density = 1510;
    mat_props.Young_modulus = 2e6;
    mat_props.Poisson_ratio = 0.3;
    mat_props.mu_I0 = 0.04;
    mat_props.mu_fric_s = 0.3;
    mat_props.mu_fric_2 = 0.48;
    mat_props.average_diam = 0.002;
    sysSPH.SetElasticSPH(mat_props);

    ChFsiFluidSystemSPH::SPHParameters sph_params;
    sph_params.integration_scheme = IntegrationScheme::RK2;
    sph_params.initial_spacing = init_spacing;
    sph_params.d0_multiplier = 1.3;
    sph_params.artificial_viscosity = 0.01;
    // Set boundary type
    if (boundary_method == "holmes") {
        sph_params.boundary_method = BoundaryMethod::HOLMES;
    } else {
        sph_params.boundary_method = BoundaryMethod::ADAMI;
    }
    // Set viscosity type
    if (viscosity_method == "artificial_bilateral") {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_BILATERAL;
    } else {
        sph_params.viscosity_method = ViscosityMethod::ARTIFICIAL_UNILATERAL;
    }
    sph_params.shifting_method = ShiftingMethod::PPST_XSPH;
    sph_params.shifting_xsph_eps = 0.5;
    sph_params.shifting_ppst_pull = 1.0;
    sph_params.shifting_ppst_push = 3.0;
    sph_params.free_surface_threshold = 0.8;
    sph_params.num_proximity_search_steps = ps_freq;
    sph_params.kernel_type = KernelType::CUBIC_SPLINE;
    sph_params.use_variable_time_step = false;
    sysSPH.SetSPHParameters(sph_params);

    // Dimension of the space domain
    double bxDim = 0.14;
    double byDim = 0.1;
    double fzDim = 0.15;
    double bzDim = fzDim;

    // Set the periodic boundary condition
    ChVector3d cMin(-bxDim / 2 - 3 * init_spacing, -byDim / 2 - 3 * init_spacing, -bzDim * 1.2);
    ////ChVector3d cMax(bxDim / 2 * 1.2, byDim / 2 * 1.2, (bzDim + Hdrop + sphere_radius + init_spacing) * 1.2);
    ChVector3d cMax(bxDim / 2 + 3 * init_spacing, byDim / 2 + 3 * init_spacing,
                    (bzDim + sphere_radius + init_spacing) * 1.2);
    sysSPH.SetComputationalDomain(ChAABB(cMin, cMax), BC_NONE);

    // Create SPH particle locations using a regular grid sampler
    chrono::utils::ChGridSampler<> sampler(init_spacing);
    ChVector3d boxCenter(0, 0, fzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - init_spacing, byDim / 2 - init_spacing, fzDim / 2 - init_spacing);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles to the fluid system
    double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
    for (const auto& p : points) {
        double pre_ini = sysSPH.GetDensity() * gz * (-p.z() + fzDim);
        double rho_ini = sysSPH.GetDensity() + pre_ini / (sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed());
        sysSPH.AddSPHParticle(p, rho_ini, pre_ini, sysSPH.GetViscosity(), ChVector3d(0));
    }

    // Create the solid domain
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.3f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Create a box body fixed to ground (used to carry collision geometry)
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(true);
    sysMBS.AddBody(box);

    // Add boundary BCE particles to the FSI system
    auto box_bce = sysSPH.CreatePointsBoxContainer(ChVector3d(bxDim, byDim, bzDim), {2, 2, -1});
    sysFSI.AddFsiBoundary(box_bce, ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT));

    // Create a falling sphere
    double volume = ChSphere::CalcVolume(sphere_radius);
    double mass = sphere_density * volume;
    ChMatrix33d inertia = mass * ChSphere::CalcGyration(sphere_radius);
    double impact_vel = std::sqrt(2 * Hdrop * g);

    ////double sphere_z_pos = Hdrop + fzDim + sphere_radius + 0.5 * init_spacing;
    ////double sphere_z_vel = 0;

    double sphere_z_pos = fzDim + sphere_radius + 0.5 * init_spacing;
    double sphere_z_vel = impact_vel;

    auto sphere = chrono_types::make_shared<ChBody>();
    sphere->SetPos(ChVector3d(0, 0, sphere_z_pos));
    sphere->SetPosDt(ChVector3d(0, 0, -sphere_z_vel));
    sphere->SetMass(mass);
    sphere->SetInertia(inertia);
    chrono::utils::AddSphereGeometry(sphere.get(), cmaterial, sphere_radius);
    sphere->EnableCollision(true);
    sphere->GetCollisionModel()->SetSafeMargin(init_spacing);
    sysMBS.AddBody(sphere);

    // Create body BCE particles and add the sphere as an FSI body
    auto sphere_bce = sysSPH.CreatePointsSphereInterior(sphere_radius, true);
    sysFSI.AddFsiBody(sphere, sphere_bce, ChFrame<>(), false);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Output directories
    std::string out_dir;
    if (output || snapshots) {
        out_dir = GetChronoOutputPath() + "FSI_Cratering/";
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cerr << "Error creating directory " << out_dir << std::endl;
            return 1;
        }

        std::stringstream ss;
        ss << viscosity_method << "_" << boundary_method;
        ss << "_ps" << ps_freq;
        ss << "_d" << sphere_density;
        ss << "_h" << Hdrop;
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
        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, impact_vel / 2);

        auto visFSI = chrono_types::make_shared<ChSphVisualizationVSG>(&sysFSI);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->SetSPHVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());
        visFSI->SetBCEVisibilityCallback(chrono_types::make_shared<MarkerPositionVisibilityCallback>());

        // VSG visual system (attach visFSI as plugin)
        auto visVSG = chrono_types::make_shared<vsg3d::ChVisualSystemVSG>();
        visVSG->AttachPlugin(visFSI);
        visVSG->AttachSystem(&sysMBS);
        visVSG->SetWindowTitle("Cratering");
        visVSG->SetWindowSize(1280, 800);
        visVSG->SetWindowPosition(100, 100);
        visVSG->AddCamera(ChVector3d(0, -3 * byDim, 0.75 * bzDim), ChVector3d(0, 0, 0.75 * bzDim));
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

    std::string out_file = out_dir + "/sphere_penetration_depth.txt";
    std::ofstream ofile(out_file, std::ios::trunc);

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        if (output && time >= out_frame / output_fps) {
            sysSPH.SaveParticleData(out_dir + "/particles");
            sysSPH.SaveSolidData(out_dir + "/fsi", time);
            out_frame++;
        }

#ifdef CHRONO_VSG
        if (render && time >= render_frame / render_fps) {
            if (!vis->Run())
                break;
            vis->Render();

            if (snapshots) {
                std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/" << std::setw(5) << std::setfill('0') << render_frame << ".jpg";
                vis->WriteImageToFile(filename.str());
            }

            render_frame++;
        }
#endif

        // Write penetration depth to file
        double d_pen = fzDim + sphere_radius + 0.5 * init_spacing - sphere->GetPos().z();
        ofile << time << " " << d_pen << " " << sphere->GetPos().x() << " " << sphere->GetPos().y() << " "
              << sphere->GetPos().z() << " " << sphere->GetPosDt().x() << " " << sphere->GetPosDt().y() << " "
              << sphere->GetPosDt().z() << std::endl;
        // Advance simulation for one timestep for all systems
        sysFSI.DoStepDynamics(dT);
        time += dT;

        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
