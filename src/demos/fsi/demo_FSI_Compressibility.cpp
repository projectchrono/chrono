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

// General Includes
#include <assert.h>
#include <stdlib.h>
#include <ctime>

// Chrono includes
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsGenerators.h"

// Chrono fsi includes
#include "chrono_fsi/ChSystemFsi.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;
using namespace chrono::fsi;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_COMPRESSIBILITY/";
std::string demo_dir;

// Save data as csv files, turn it on to be able to see the results off-line using paraview
bool save_output = true;

// Size of the box
double bxDim = 1.0;
double byDim = 1.0;
double bzDim = 1.4;

// Size of the fluid domain
double fxDim = bxDim;
double fyDim = byDim;
double fzDim = 1.0;

// -----------------------------------------------------------------
// Show command line usage
// -----------------------------------------------------------------
void ShowUsage() {
    std::cout << "usage: ./demo_FSI_Compressibility <json_file>" << std::endl;
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFilesMBD(ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int next_frame,
                          double mTime) {
    double frame_time = 1.0 / paramsH->out_fps;
    static int out_frame = 0;

    if (save_output && std::abs(mTime - (next_frame)*frame_time) < 1e-5) {
        myFsiSystem.PrintParticleToFile(demo_dir);

        std::cout << "-------------------------------------\n" << std::endl;
        std::cout << "             Output frame:   " << next_frame << std::endl;
        std::cout << "             Time:           " << mTime << std::endl;
        std::cout << "-------------------------------------\n" << std::endl;

        out_frame++;
    }
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH) {
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();

    // Set common material Properties
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);
    // Ground body
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->GetCollisionModel()->ClearModel();

    auto initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom and top wall
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 2 * initSpace0);
    ChVector<> pos_zn(0, 0, -2 * initSpace0);

    // Left and right wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);

    // Front and back wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yn, QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

    myFsiSystem.AddBceBox(paramsH, ground, pos_zn, QUNIT, size_XY, 12);
    myFsiSystem.AddBceBox(paramsH, ground, pos_xp, QUNIT, size_YZ, 23);
    myFsiSystem.AddBceBox(paramsH, ground, pos_xn, QUNIT, size_YZ, 23);
    myFsiSystem.AddBceBox(paramsH, ground, pos_yp, QUNIT, size_XZ, 13);
    myFsiSystem.AddBceBox(paramsH, ground, pos_yn, QUNIT, size_XZ, 13);
}

// =============================================================================

int main(int argc, char* argv[]) {
    // Create a physic system to handle multibody dynamics
    ChSystemSMC mphysicalSystem;

    // Create an FSI system to handle fluid dynamics
    ChSystemFsi myFsiSystem(mphysicalSystem);

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Compressibility_Explicit.json");
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

    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);

    auto initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    ChVector<> cMin = ChVector<>(-bxDim / 2, -byDim / 2, -bzDim / 2) - ChVector<>(initSpace0 * 20);
    ChVector<> cMax = ChVector<>(bxDim / 2, byDim / 2, bzDim) + ChVector<>(initSpace0 * 10);
    myFsiSystem.SetBoundaries(cMin, cMax, paramsH);

    // Setup sub doamins for a faster neighbor particle searching
    myFsiSystem.SetSubDomain(paramsH);

    // Setup the output directory for FSI data
    myFsiSystem.SetFsiOutputDir(paramsH, demo_dir, out_dir, inputJson);

    // Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);

    // Use a chrono sampler to create a bucket of granular material
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0 * initSpace0, fzDim / 2 + 1 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.AddSphMarker(points[i], paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, paramsH->HSML, -1);
    }
    myFsiSystem.AddRefArray(0, (int)numPart, -1, -1);

    // Create MBD and BCE particles for the solid domain
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);
    myFsiSystem.SetInitPressure(paramsH, fzDim);

    // Construction of the FSI system must be finalized
    myFsiSystem.Finalize();

    SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, 0, 0);

    double time = 0;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    double Global_max_dT = paramsH->dT_Max;
    double TIMING_sta = clock();
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int next_frame = (int)floor((time + 1e-6) / frame_time) + 1;
        double next_frame_time = next_frame * frame_time;
        double max_allowable_dt = next_frame_time - time;
        if (max_allowable_dt > 1e-6)
            paramsH->dT_Max = std::min(Global_max_dT, max_allowable_dt);
        else
            paramsH->dT_Max = Global_max_dT;

        myFsiSystem.DoStepDynamics_FSI();

        auto rhoPresMu = myFsiSystem.GetParticlePosOrProperties();
        auto vel = myFsiSystem.GetParticleVel();

        std::ofstream output;
        std::string delim = ",";
        output.open((demo_dir + "/Analysis.txt"), std::ios::app);
        if (tStep == 0)
            output << "Time" << delim << "Rho_fluid" << delim << "k_fluid" << std::endl;

        double KE = 0;
        double Rho = 0;
        for (int i = 0; i < numPart; i++) {
            KE += 0.5 * vel[i].Length();
            Rho += rhoPresMu[i].x();
        }

        output << time << delim << Rho / numPart << delim << paramsH->markerMass * KE / numPart << std::endl;
        output.close();
        time += paramsH->dT;
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);
        if (time > paramsH->tFinal)
            break;
    }

    // Total computational cost
    double TIMING_end = (clock() - TIMING_sta) / (double)CLOCKS_PER_SEC;
    printf("\nSimulation Finished in %f (s)\n", TIMING_end);
    
    return 0;
}
