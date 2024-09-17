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
// Author: Milad Rakhsha, Wei Hu
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_fsi/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/cxxopts/ChCLI.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;

//------------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Dam_Break";

// Output frequency
bool output = true;
double output_fps = 20;

// Dimension of the space domain
double bxDim = 6.0;
double byDim = 1.0;
double bzDim = 4.0;

// Dimension of the fluid domain
double fxDim = 2.0;
double fyDim = 1.0;
double fzDim = 2.0;

// Final simulation time
double t_end = 10.0;

// Enable/disable run-time visualization
bool render = true;
double render_fps = 100;

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // General setting of ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sysMBS.AddBody(ground);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 0, 2));
}

// =============================================================================

bool GetProblemSpecs(int argc,
                     char** argv,
                     std::string& inputJSON,
                     double& t_end,
                     bool& verbose,
                     bool& output,
                     double& output_fps,
                     bool& render,
                     double& render_fps,
                     bool& snapshots,
                     int& ps_freq) {
    ChCLI cli(argv[0], "Dam Break FSI demo");

    cli.AddOption<std::string>("Input", "inputJSON", "Problem specification file [JSON format]", inputJSON);
    cli.AddOption<double>("Input", "t_end", "Simulation duration [s]", std::to_string(t_end));

    cli.AddOption<bool>("Output", "quiet", "Disable verbose terminal output");

    cli.AddOption<bool>("Output", "output", "Enable collection of output files");
    cli.AddOption<double>("Output", "output_fps", "Output frequency [fps]", std::to_string(output_fps));

    cli.AddOption<bool>("Visualization", "no_vis", "Disable run-time visualization");
    cli.AddOption<double>("Visualization", "render_fps", "Render frequency [fps]", std::to_string(render_fps));
    cli.AddOption<bool>("Visualization", "snapshots", "Enable writing snapshot image files");

    cli.AddOption<int>("Proximity Search", "ps_freq", "Frequency of Proximity Search", std::to_string(ps_freq));

    if (!cli.Parse(argc, argv)) {
        cli.Help();
        return false;
    }

    inputJSON = cli.Get("inputJSON").as<std::string>();
    t_end = cli.GetAsType<double>("t_end");

    verbose = !cli.GetAsType<bool>("quiet");
    output = cli.GetAsType<bool>("output");
    render = !cli.GetAsType<bool>("no_vis");
    snapshots = cli.GetAsType<bool>("snapshots");

    output_fps = cli.GetAsType<double>("output_fps");
    render_fps = cli.GetAsType<double>("render_fps");

    ps_freq = cli.GetAsType<int>("ps_freq");

    return true;
}

int main(int argc, char* argv[]) {
    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_DamBreak_Explicit.json");
    bool verbose = true;
    bool snapshots = false;
    int ps_freq = 1;
    if (!GetProblemSpecs(argc, argv, inputJson, t_end, verbose, output, output_fps, render, render_fps, snapshots,
                         ps_freq)) {
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysFSI.SetVerbose(verbose);

    // Use the specified input JSON file
    sysFSI.ReadParametersFromFile(inputJson);

    out_dir = out_dir + std::to_string(ps_freq);

    // Set up the periodic boundary condition (only in Y direction)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-bxDim / 2 - 10.0 * initSpace0, -byDim / 2 - 1.0 * initSpace0 / 2.0, -2.0 * bzDim);
    ChVector3d cMax = ChVector3d(bxDim / 2 + 10.0 * initSpace0, byDim / 2 + 1.0 * initSpace0 / 2.0, 2.0 * bzDim);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create Fluid region and discretize with SPH particles
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - initSpace0, fyDim / 2, fzDim / 2 - initSpace0);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    double gz = std::abs(sysFSI.GetGravitationalAcceleration().z());
    for (int i = 0; i < numPart; i++) {
        // Calculate the pressure of a steady state (p = rho*g*h)
        auto pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + fzDim);
        auto rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSPHParticle(points[i], rho_ini, pre_ini, sysFSI.GetViscosity());
    }

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);
    sysFSI.SetNumProximitySearchSteps(ps_freq);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    std::cout << "Neighbor search steps: " << sysFSI.GetNumProximitySearchSteps() << std::endl;
    if (output || snapshots) {
        if (output) {
            // Create output directories
            if (!filesystem::create_directory(filesystem::path(out_dir))) {
                std::cerr << "Error creating directory " << out_dir << std::endl;
                return 1;
            }
            out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();

            std::cout << "Output directory: " << out_dir << std::endl;

            if (!filesystem::create_directory(filesystem::path(out_dir))) {
                std::cerr << "Error creating directory " << out_dir << std::endl;
                return 1;
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
                std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
                return 1;
            }
        }
        if (snapshots) {
            if (!output) {
                // Create output directories
                if (!filesystem::create_directory(filesystem::path(out_dir))) {
                    std::cerr << "Error creating directory " << out_dir << std::endl;
                    return 1;
                }
                out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();

                std::cout << "Output directory: " << out_dir << std::endl;

                if (!filesystem::create_directory(filesystem::path(out_dir))) {
                    std::cerr << "Error creating directory " << out_dir << std::endl;
                    return 1;
                }
            }
            if (!filesystem::create_directory(filesystem::path(out_dir + "/snapshots"))) {
                std::cerr << "Error creating directory " << out_dir + "/snapshots" << std::endl;
                return 1;
            }
        }
    }

    // Create a run-tme visualizer
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif
#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle("Chrono::FSI dam break");
        visFSI->AddCamera(ChVector3d(0, -6 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.4 * bzDim));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, 5.0));
        visFSI->Initialize();
    }

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    double time = 0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        std::cout << "step: " << sim_frame << "  time: " << time << std::endl;

        // Save data of the simulation
        if (output && time >= out_frame / output_fps) {
            std::cout << "------- OUTPUT" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;
            if (snapshots) {
                if (verbose)
                    std::cout << " -- Snapshot frame " << render_frame << " at t = " << time << std::endl;
                std::ostringstream filename;
                filename << out_dir << "/snapshots/img_" << std::setw(5) << std::setfill('0') << render_frame + 1
                         << ".bmp";
                visFSI->GetVisualSystem()->WriteImageToFile(filename.str());
            }
            render_frame++;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();

        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "End Time: " << t_end << std::endl;
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
