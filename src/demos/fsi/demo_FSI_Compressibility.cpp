// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_fsi/sph/ChFluidSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
std::string out_dir = GetChronoOutputPath() + "FSI_Compressibility";

// Output frequency
bool output = true;
double output_fps = 20;

// Size of the box
double smalldis = 1.0e-5;
double bxDim = 1.0 + smalldis;
double byDim = 1.0 + smalldis;
double bzDim = 1.4 + smalldis;

// Size of the fluid domain
double fxDim = bxDim;
double fyDim = byDim;
double fzDim = 1.0 + smalldis;

// Final simulation time
double t_end = 2.0;

// Enable/disable run-time visualization
bool render = true;
float render_fps = 1000;

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChFluidSystemSPH& sysSPH) {
    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    ground->EnableCollision(false);
    sysMBS.AddBody(ground);

    // Container BCE markers
    sysSPH.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 2, -1));
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create a physics system and an SPH system
    ChSystemSMC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Compressibility_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_Compressibility <json_file>" << std::endl;
        return 1;
    }
    sysSPH.ReadParametersFromFile(inputJson);

    auto initSpace0 = sysSPH.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-bxDim / 2, -byDim / 2, -bzDim / 2) - ChVector3d(initSpace0 * 20);
    ChVector3d cMax = ChVector3d(bxDim / 2, byDim / 2, bzDim) + ChVector3d(initSpace0 * 10);
    sysSPH.SetBoundaries(cMin, cMax);

    // Create an initial box for the terrain patch
    chrono::utils::ChGridSampler<> sampler(initSpace0);

    // Use a chrono sampler to create a bucket of granular material
    ChVector3d boxCenter(0, 00, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2 - initSpace0, fyDim / 2 - initSpace0, fzDim / 2 - initSpace0);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysSPH.AddSPHParticle(points[i]);
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysSPH);
    sysSPH.SetInitPressure(fzDim);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysSPH.GetPhysicsProblemString() + "_" + sysSPH.GetSphMethodTypeString();
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
            std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
            return 1;
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
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysSPH);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysSPH);
#endif
                break;
            }
        }

        auto col_callback = chrono_types::make_shared<ParticleVelocityColorCallback>(0, 5.0);

        visFSI->SetTitle("Chrono::FSI compressibility test");
        visFSI->AddCamera(ChVector3d(0, -5 * byDim, 0.5 * bzDim), ChVector3d(0, 0, 0.5 * bzDim));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableFluidMarkers(true);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(col_callback);
        visFSI->Initialize();
    }

    // Start the simulation
    std::ofstream outf;
    std::string delim = ",";
    outf.open((out_dir + "/Analysis.txt"), std::ios::trunc);
    outf << "Time" << delim << "Rho_fluid" << delim << "k_fluid" << std::endl;

    double dT = sysSPH.GetStepSize();
    double time = 0.0;
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
            sysSPH.SaveParticleData(out_dir + "/particles");
            out_frame++;
        }

        // Render FSI system
        if (render && time >= render_frame / render_fps) {
            if (!visFSI->Render())
                break;
            render_frame++;
        }

        sysFSI.DoStepDynamics(dT);

        auto rhoPresMu = sysSPH.GetParticleFluidProperties();
        auto vel = sysSPH.GetParticleVelocities();

        double KE = 0;
        double Rho = 0;
        for (int i = 0; i < numPart; i++) {
            KE += 0.5 * vel[i].Length();
            Rho += rhoPresMu[i].x();
        }

        outf << time << delim << Rho / numPart << delim << sysSPH.GetParticleMass() * KE / numPart << std::endl;

        time += dT;
        sim_frame++;
    }

    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
