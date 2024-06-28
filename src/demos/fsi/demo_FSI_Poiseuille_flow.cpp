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

using namespace chrono;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Poiseuille_Flow/";

// Output frequency
bool output = true;
double out_fps = 20;

// Dimension of the space domain
double bxDim = 0.2;
double byDim = 0.1;
double bzDim = 0.2;

// Final simulation time
double t_end = 10.0;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 100;

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
                              ChVector3i(0, 0, 2));
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }

    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_Poiseuille_flow <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    // Set the periodic boundary condition (in X and Y direction)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, -5.0 * initSpace0);
    ChVector3d cMax = ChVector3d(bxDim / 2 + initSpace0 / 2, byDim / 2 + initSpace0 / 2, bzDim + 5.0 * initSpace0);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create Fluid region and discretize with SPH particles
    ChVector3d boxCenter(0.0, 0.0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2 - initSpace0);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysFSI.AddSPHParticle(points[i]);
    }

    // Create solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();
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

        visFSI->SetTitle("Chrono::FSI Poiseuille flow");
        visFSI->AddCamera(ChVector3d(0, -5 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.5 * bzDim));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(chrono_types::make_shared<VelocityColorCallback>(0, 0.04));
        visFSI->Initialize();
    }

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));

    double time = 0;
    int current_step = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        std::cout << "step: " << current_step << "  time: " << time << std::endl;

        // Save data of the simulation
        if (output && current_step % output_steps == 0) {
            std::cout << "------- OUTPUT" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
        }

        // Render FSI system
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();

        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
