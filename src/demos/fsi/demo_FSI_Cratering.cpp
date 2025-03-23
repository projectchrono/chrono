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
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
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
class MarkerPositionVisibilityCallback : public ChFsiVisualizationVSG::MarkerVisibilityCallback {
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
                     std::string& boundary_type,
                     std::string& viscosity_type) {
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

    cli.AddOption<std::string>("Physics", "boundary_type", "Boundary condition type (holmes/adami)", "adami");
    cli.AddOption<std::string>("Physics", "viscosity_type",
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

    boundary_type = cli.GetAsType<std::string>("boundary_type");
    viscosity_type = cli.GetAsType<std::string>("viscosity_type");

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
    std::string boundary_type = "adami";
    std::string viscosity_type = "artificial_unilateral";

    // Parse command-line arguments
    if (!GetProblemSpecs(argc, argv, t_end, verbose, output, output_fps, snapshots, ps_freq, sphere_density, Hdrop,
                         render, boundary_type, viscosity_type)) {
        return 1;
    }

    // Create a physics system
    ChSystemSMC sysMBS;

    // Create a fluid system
    ChFsiFluidSystemSPH sysSPH;
    // Create an FSI system
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    // Set boundary type
    if (boundary_type == "holmes") {
        sysSPH.SetBoundaryType(BoundaryType::HOLMES);
    } else {
        sysSPH.SetBoundaryType(BoundaryType::ADAMI);
    }

    // Set viscosity type
    if (viscosity_type == "artificial_bilateral") {
        sysSPH.SetViscosityType(ViscosityType::ARTIFICIAL_BILATERAL);
    } else {
        sysSPH.SetViscosityType(ViscosityType::ARTIFICIAL_UNILATERAL);
    }

    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Cratering_granular.json");
    sysSPH.ReadParametersFromFile(inputJson);
    auto init_spacing = sysSPH.GetInitialSpacing();

    double g = 9.81;
    sysSPH.SetGravitationalAcceleration(ChVector3d(0, 0, -g));
    sysMBS.SetGravitationalAcceleration(sysSPH.GetGravitationalAcceleration());

    sysFSI.SetVerbose(verbose);
    sysSPH.SetNumProximitySearchSteps(ps_freq);

    sysSPH.SetShiftingMethod(ShiftingMethod::PPST_XSPH);
    sysSPH.SetShiftingPPSTParameters(3.0, 0.0);
    sysSPH.SetShiftingXSPHParameters(0.25);

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
    sysSPH.SetComputationalBoundaries(cMin, cMax, PeriodicSide::NONE);

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

    // Create MBD and BCE particles for the solid domain
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.3f);
    cmaterial->SetRestitution(0.05f);
    cmaterial->SetAdhesion(0);

    // Create a container
    auto box = chrono_types::make_shared<ChBody>();
    box->SetPos(ChVector3d(0.0, 0.0, 0.0));
    box->SetRot(ChQuaternion<>(1, 0, 0, 0));
    box->SetFixed(true);
    sysMBS.AddBody(box);

    // Add collision geometry for the container walls
    chrono::utils::AddBoxContainer(box, cmaterial,                                 //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(2, 2, -1),                           //
                                   false);
    box->EnableCollision(false);

    // Add BCE particles attached on the walls into FSI system
    sysSPH.AddBoxContainerBCE(box,                                            //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 2, -1));

    // Create a falling sphere
    double volume = ChSphere::GetVolume(sphere_radius);
    double mass = sphere_density * volume;
    auto inertia = mass * ChSphere::GetGyration(sphere_radius);
    double impact_vel = std::sqrt(2 * Hdrop * g);

    ////double sphere_z_pos = Hdrop + fzDim + sphere_radius + 0.5 * init_spacing;
    ////double sphere_z_vel = 0;

    double sphere_z_pos = fzDim + sphere_radius + 0.5 * init_spacing;
    double sphere_z_vel = impact_vel;

    auto sphere = chrono_types::make_shared<ChBody>();
    sysMBS.AddBody(sphere);
    sphere->SetPos(ChVector3d(0, 0, sphere_z_pos));
    sphere->SetPosDt(ChVector3d(0, 0, -sphere_z_vel));
    sphere->SetMass(mass);
    sphere->SetInertia(inertia);

    chrono::utils::AddSphereGeometry(sphere.get(), cmaterial, sphere_radius);
    sphere->GetCollisionModel()->SetSafeMargin(init_spacing);

    sysFSI.AddFsiBody(sphere);
    sysSPH.AddSphereBCE(sphere, ChFrame<>(VNULL, QUNIT), sphere_radius, true, true);

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
        ss << viscosity_type << "_" << boundary_type;
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

        auto visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
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
        visVSG->SetWindowSize(1280, 720);
        visVSG->SetWindowPosition(400, 400);
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