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
const std::string out_dir = GetChronoOutputPath() + "FSI_DAM_BREAK/";
std::string demo_dir;

// Save data as csv files to see the results off-line using Paraview
bool save_output = true;

// Output frequency
double out_fps = 20;

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

void ShowUsage() {
    std::cout << "usage: ./demo_FSI_DamBreak <json_file>" << std::endl;
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFilesMBD(ChSystemFsi& sysFSI,
                          ChSystemSMC& sysMBS,
                          int this_frame,
                          double mTime) {
    // Simulation time between two output frames
    double frame_time = 1.0 / out_fps;

    // Output data to files
    if (save_output && std::abs(mTime - (this_frame)*frame_time) < 1e-5) {
        sysFSI.PrintParticleToFile(demo_dir);
        std::cout << "\n--------------------------------\n" << std::endl;
        std::cout << "------------ Output Frame:   " << this_frame << std::endl;
        std::cout << "------------ Sim Time:       " << mTime << " (s)\n" << std::endl;
        std::cout << "--------------------------------\n" << std::endl;
    }
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS,
                      ChSystemFsi& sysFSI) {
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
    auto initSpace0 = sysFSI.GetInitialSpacing();

    // Bottom and top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 0 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 0 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    // Front and back wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    // Add the walls into chrono system
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XY, pos_zn, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xn, QUNIT, true);
    // You may uncomment the following lines to have side walls as well.
    // To show the use of Periodic boundary condition, these walls are not added
    // To this end, cMin and cMax were set up appropriately
    // chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yp, QUNIT, true);
    // chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yn, QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    sysMBS.AddBody(ground);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBceBox(ground, pos_zn, QUNIT, size_XY, 12);
    sysFSI.AddBceBox(ground, pos_zp, QUNIT, size_XY, 12);
    sysFSI.AddBceBox(ground, pos_xp, QUNIT, size_YZ, 23);
    sysFSI.AddBceBox(ground, pos_xn, QUNIT, size_YZ, 23);
    // If you uncommented the above lines that add the side walls, you should uncomment the following two lines as
    // well. This is necessary in order to populate the walls with BCE markers for the fluid simulation
    // sysFSI.AddBoxBce(ground, pos_yp, QUNIT, size_XZ, 13);
    // sysFSI.AddBoxBce(ground, pos_yn, QUNIT, size_XZ, 13);
}

// =============================================================================
int main(int argc, char* argv[]) {
    // Create a physics system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(sysMBS);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_DamBreak_Explicit.json");
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
    sysFSI.ReadParametersFromFile(inputJson, ChVector<>(bxDim, byDim, bzDim));

    // Dimension of the space domain
    ChVector<> bDim = sysFSI.GetContainerDim();
    bxDim = bDim.x();
    byDim = bDim.y();
    bzDim = bDim.z();

    // Dimension of the fluid domain
    ChVector<> fDim = sysFSI.GetSimDim();
    fxDim = fDim.x();
    fyDim = fDim.y();
    fzDim = fDim.z();

    // Set up the periodic boundary condition (only in Y direction)
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector<> cMin = ChVector<>(-bxDim / 2 - 10.0 * initSpace0, -byDim / 2 - 1.0 * initSpace0 / 2.0, -2.0 * bzDim);
    ChVector<> cMax = ChVector<>(bxDim / 2 + 10.0 * initSpace0, byDim / 2 + 1.0 * initSpace0 / 2.0, 2.0 * bzDim);
    sysFSI.SetBoundaries(cMin, cMax);

    // Setup the output directory for FSI data
    sysFSI.SetFsiOutputDir(demo_dir, out_dir, inputJson);

    // Create Fluid region and discretize with SPH particles
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0.0, fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);

    // Use a chrono sampler to create a bucket of points
    chrono::utils::GridSampler<> sampler(initSpace0);
    chrono::utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add fluid particles from the sampler points to the FSI system
    size_t numPart = points.size();
    double gz = std::abs(sysFSI.Get_G_acc().z());
    for (int i = 0; i < numPart; i++) {
        // Calculate the pressure of a steady state (p = rho*g*h)
        auto pre_ini = sysFSI.GetDensity() * gz * (-points[i].z() + fzDim);
        auto rho_ini = sysFSI.GetDensity() + pre_ini / (sysFSI.GetSoundSpeed() * sysFSI.GetSoundSpeed());
        sysFSI.AddSphMarker(points[i], rho_ini, pre_ini, sysFSI.GetViscosity(), sysFSI.GetKernelLength(), -1);
    }
    sysFSI.AddRefArray(0, (int)numPart, -1, -1);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Start the simulation
    double dT = sysFSI.GetStepSize();
    double time = 0;
    int stepEnd = int(t_end / dT);
    double TIMING_sta = clock();
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / out_fps;
        int this_frame = (int)floor((time + 1e-6) / frame_time);

        // Save data of the simulation
        SaveParaViewFilesMBD(sysFSI, sysMBS, this_frame, time);

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
    }

    // Total computational cost
    double TIMING_end = (clock() - TIMING_sta) / (double)CLOCKS_PER_SEC;
    printf("\nSimulation Finished in %f (s)\n", TIMING_end);

    return 0;
}
