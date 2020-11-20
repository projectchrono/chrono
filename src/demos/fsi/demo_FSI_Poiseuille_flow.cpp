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
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJSON.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

// Chrono namespaces
using namespace chrono;
using namespace collision;

using std::cout;
using std::endl;
std::ofstream simParams;
typedef fsi::Real Real;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_POISEUILLE_FLOW/";
std::string demo_dir;

// Save data as csv files to see the results off-line using Paraview
bool save_output = true;

// Dimension of the space domain
Real bxDim;
Real byDim;
Real bzDim;
// Dimension of the fluid domain
Real fxDim;
Real fyDim;
Real fzDim;

void ShowUsage() {
    cout << "usage: ./demo_FSI_Poiseuille_flow <json_file>" << endl;
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int this_frame,
                          double mTime) {
    // Simulation steps between two output frames
    int out_steps = (int)ceil((1.0 / paramsH->dT) / paramsH->out_fps);

    // Simulation time between two output frames
    double frame_time = 1.0 / paramsH->out_fps;
    
    // Output data to files
    if (save_output && std::abs(mTime - (this_frame)*frame_time) < 1e-9) {
        fsi::utils::PrintToFile(
            myFsiSystem.GetDataManager()->sphMarkersD2->posRadD, 
            myFsiSystem.GetDataManager()->sphMarkersD2->velMasD,
            myFsiSystem.GetDataManager()->sphMarkersD2->rhoPresMuD,
            myFsiSystem.GetDataManager()->fsiGeneralData->sr_tau_I_mu_i,
            myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray, 
            thrust::host_vector<int4>(), demo_dir, true);
        cout << "\n--------------------------------\n" << endl;
        cout << "------------ Output Frame:   " << this_frame << endl;
        cout << "------------ Sim Time:       " << mTime << " (s)\n" <<endl;
        cout << "--------------------------------\n" << endl;
    }
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI, 
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
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
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom and Top wall - size and position
    ChVector<> sizeBottom(bxDim / 2, byDim / 2 + 0 * initSpace0, 2 * initSpace0);
    ChVector<> sizeTop = sizeBottom;
    ChVector<> posBottom(0, 0, -3 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 1 * initSpace0);

    // Add the walls into chrono system
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, sizeBottom, posBottom, chrono::QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, sizeTop, posTop, chrono::QUNIT, true);

    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

    // Add BCE particles attached on the walls into FSI system
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posBottom, chrono::QUNIT, sizeBottom);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posTop, chrono::QUNIT, sizeBottom);

}

// =============================================================================
int main(int argc, char* argv[]) {
    // Create a physics system and an FSI system
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(mphysicalSystem);

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Poiseuille_flow_Explicit.json");
    if (argc == 1) {
        fsi::utils::ParseJSON(inputJson, paramsH, fsi::mR3(bxDim, byDim, bzDim));
    } else if (argc == 2) {
        fsi::utils::ParseJSON(argv[1], paramsH, fsi::mR3(bxDim, byDim, bzDim));
        std::string input_json = std::string(argv[1]);
        inputJson = GetChronoDataFile(input_json);
    } else {
        ShowUsage();
        return 1;
    }

    // Dimension of the space domain
    bxDim = paramsH->boxDimX;
    byDim = paramsH->boxDimY;
    bzDim = paramsH->boxDimZ;
    // Dimension of the fluid domain
    fxDim = paramsH->fluidDimX;
    fyDim = paramsH->fluidDimY;
    fzDim = paramsH->fluidDimZ;

    // Set the periodic boundary condition (in X and Y direction)
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = fsi::mR3(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, 0.0   - 5.0 * initSpace0);
    paramsH->cMax = fsi::mR3( bxDim / 2 + initSpace0 / 2,  byDim / 2 + initSpace0 / 2, bzDim + 5.0 * initSpace0);

    // Set the time integration type and the linear solver type (only for ISPH)
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);

    // Call FinalizeDomain to setup the binning for neighbor search
    fsi::utils::FinalizeDomain(paramsH);
    fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);

    // Create Fluid region and discretize with SPH particles
    ChVector<> boxCenter(0.0, 0.0, fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

    // Use a chrono sampler to create a bucket of points
    utils::GridSampler<> sampler(initSpace0);
    utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.GetDataManager()->AddSphMarker(fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
                                                   fsi::mR3(1e-10),
                                                   fsi::mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1));
    }
    myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, (int)numPart, -1, -1));

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    // Construction of the FSI system must be finalized before running
    myFsiSystem.Finalize();

    // Start the simulation
    Real time = 0;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    double TIMING_sta = clock();
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int this_frame = (int)floor((time + 1e-9) / frame_time);

        // Save data of the simulation
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, this_frame, time);
        
        // Call the FSI solver
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
    }

    // Total computational cost
    double TIMING_end = (clock() - TIMING_sta) / (double)CLOCKS_PER_SEC;
    printf("\nSimulation Finished in %f (s)\n", TIMING_end);

    return 0;
}
