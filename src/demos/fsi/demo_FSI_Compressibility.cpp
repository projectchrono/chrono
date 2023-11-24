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

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Compressibility/";

// Output frequency
bool output = true;
double out_fps = 20;

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

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(false);
    sysMBS.AddBody(ground);

    // Container BCE markers
    sysFSI.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector<>(0, 0, bzDim / 2), QUNIT),  //
                              ChVector<>(bxDim, byDim, bzDim),                //
                              ChVector<int>(2, 2, -1));
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

    // Create a physic system to handle multibody dynamics
    ChSystemSMC sysMBS;

    // Create an FSI system to handle fluid dynamics
    ChSystemFsi sysFSI(&sysMBS);

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
    sysFSI.ReadParametersFromFile(inputJson);

    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin = ChVector<>(-bxDim / 2, -byDim / 2, -bzDim / 2) - ChVector<>(initSpace0 * 20);
    ChVector<> cMax = ChVector<>(bxDim / 2, byDim / 2, bzDim) + ChVector<>(initSpace0 * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);

    // Use a chrono sampler to create a bucket of granular material
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0 * initSpace0, fzDim / 2 + 1 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysFSI.AddSPHParticle(points[i]);
    }

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(sysMBS, sysFSI);
    sysFSI.SetInitPressure(fzDim);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));

    std::ofstream outf;
    std::string delim = ",";
    outf.open((out_dir + "/Analysis.txt"), std::ios::trunc);
    outf << "Time" << delim << "Rho_fluid" << delim << "k_fluid" << std::endl;

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

        sysFSI.DoStepDynamics_FSI();

        auto rhoPresMu = sysFSI.GetParticleFluidProperties();
        auto vel = sysFSI.GetParticleVelocities();

        double KE = 0;
        double Rho = 0;
        for (int i = 0; i < numPart; i++) {
            KE += 0.5 * vel[i].Length();
            Rho += rhoPresMu[i].x();
        }

        outf << time << delim << Rho / numPart << delim << sysFSI.GetParticleMass() * KE / numPart << std::endl;

        time += dT;
        current_step++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}
