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

// General Includes
#include <assert.h>
#include <stdlib.h>
#include <ctime>

// Chrono includes
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

// Chrono fsi includes
#include "chrono_fsi/ChSystemFsi.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::fsi;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_POISEUILLE_FLOW/";
std::string demo_dir;

// Save data as csv files to see the results off-line using Paraview
bool save_output = true;

// Dimension of the space domain
double bxDim = 0.2;
double byDim = 0.1;
double bzDim = 0.2;
// Dimension of the fluid domain
double fxDim = 0.2;
double fyDim = 0.1;
double fzDim = 0.2;

void ShowUsage() {
    std::cout << "usage: ./demo_FSI_Poiseuille_flow <json_file>" << std::endl;
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFilesMBD(ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int next_frame,
                          double mTime) {
    // Simulation time between two output frames
    double frame_time = 1.0 / paramsH->out_fps;
    
    // Output data to files
    if (save_output && std::abs(mTime - (next_frame)*frame_time) < 1e-6) {
        myFsiSystem.PrintParticleToFile(demo_dir);

        std::cout << "\n--------------------------------\n" << std::endl;
        std::cout << "------------ Output Frame:   " << next_frame << std::endl;
        std::cout << "------------ Sim Time:       " << mTime << " (s)\n" << std::endl;
        std::cout << "--------------------------------\n" << std::endl;
    }
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI, 
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH) {
    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // General setting of ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->GetCollisionModel()->ClearModel();

    // Create the geometry of the boundaries
    auto initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom and top wall - size and position
    ChVector<> sizeBottom(bxDim / 2, byDim / 2 + 0 * initSpace0, 2 * initSpace0);
    ChVector<> sizeTop = sizeBottom;
    ChVector<> posBottom(0, 0, -3 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 1 * initSpace0);

    // Add the walls into chrono system
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, sizeBottom, posBottom, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, sizeTop, posTop, QUNIT, true);

    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

    // Add BCE particles attached on the walls into FSI system
    myFsiSystem.AddBceBox(paramsH, ground, posBottom, QUNIT, sizeBottom);
    myFsiSystem.AddBceBox(paramsH, ground, posTop, QUNIT, sizeBottom);

}

// =============================================================================
int main(int argc, char* argv[]) {
    // Create a physics system and an FSI system
    ChSystemSMC mphysicalSystem;
    ChSystemFsi myFsiSystem(mphysicalSystem);

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        ShowUsage();
        return 1;
    }
    myFsiSystem.SetSimParameter(inputJson, paramsH, ChVector<>(bxDim, byDim, bzDim));

    // Dimension of the space domain
    bxDim = paramsH->boxDimX;
    byDim = paramsH->boxDimY;
    bzDim = paramsH->boxDimZ;
    // Dimension of the fluid domain
    fxDim = paramsH->fluidDimX;
    fyDim = paramsH->fluidDimY;
    fzDim = paramsH->fluidDimZ;

    // Set the time integration type and the linear solver type (only for ISPH)
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);

    // Set the periodic boundary condition (in X and Y direction)
    auto initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    ChVector<> cMin = ChVector<>(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, -5.0 * initSpace0);
    ChVector<> cMax = ChVector<>( bxDim / 2 + initSpace0 / 2,  byDim / 2 + initSpace0 / 2, bzDim + 5.0 * initSpace0);
    myFsiSystem.SetBoundaries(cMin, cMax, paramsH);

    // Setup sub doamins for a faster neighbor particle searching
    myFsiSystem.SetSubDomain(paramsH);

    // Setup the output directory for FSI data
    myFsiSystem.SetFsiOutputDir(paramsH, demo_dir, out_dir, inputJson);

    // Create Fluid region and discretize with SPH particles
    ChVector<> boxCenter(0.0, 0.0, fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::GridSampler<> sampler(initSpace0);
    chrono::utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.AddSphMarker(points[i], paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, paramsH->HSML, -1);
    }
    myFsiSystem.AddRefArray(0, (int)numPart, -1, -1);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    // Construction of the FSI system must be finalized before running
    myFsiSystem.Finalize();

    // Start the simulation
    double time = 0;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    double TIMING_sta = clock();
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int next_frame = (int)floor((time + 1e-6) / frame_time) + 1;

        // Call the FSI solver
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
        
        // Save data of the simulation
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);
    }

    // Total computational cost
    double TIMING_end = (clock() - TIMING_sta) / (double)CLOCKS_PER_SEC;
    printf("\nSimulation Finished in %f (s)\n", TIMING_end);

    return 0;
}
