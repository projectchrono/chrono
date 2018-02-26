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
// Author: Milad Rakhsha
// =============================================================================

// General Includes
#include <assert.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <stdlib.h>
#include <string>
#include <vector>

// Chrono Parallel Includes
#include "chrono_parallel/physics/ChSystemParallel.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/core/ChFileutils.h"

// Chrono fsi includes
#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChFsiTypeConvert.h"
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.h"

// FSI Interface Includes
#include "demos/fsi/demo_FSI_DamBreak.h"

#define haveFluid 1

// Chrono namespaces
using namespace chrono;
using namespace chrono::collision;

using std::cout;
using std::endl;
std::ofstream simParams;

//----------------------------
// output directories and settings
//----------------------------
const std::string out_dir = GetChronoOutputPath() + "FSI_DAM_BREAK";
const std::string demo_dir = out_dir + "/DamBreak";
// Save data as csv files, turn it on to be able to see the results off-line using paraview
bool save_output = true;
// Frequency of the save output
int out_fps = 50;
typedef fsi::Real Real;

Real contact_recovery_speed = 1;  ///< recovery speed for MBD

// Dimension of the domain
Real bxDim = 5;
Real byDim = 0.5;
Real bzDim = 2.5;

// Dimension of the fluid domain
Real fxDim = 2;
Real fyDim = byDim;
Real fzDim = 2;

// Forward declarations
void SetArgumentsForMbdFromInput(int argc,
                                 char* argv[],
                                 int& threads,
                                 int& max_iteration_sliding,
                                 int& max_iteration_bilateral,
                                 int& max_iteration_normal,
                                 int& max_iteration_spinning);

void InitializeMbdPhysicalSystem(ChSystemParallelNSC& mphysicalSystem, ChVector<> gravity, int argc, char* argv[]);

void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemParallelNSC& mphysicalSystem,
                          chrono::fsi::SimParams* paramsH,
                          int tStep,
                          double mTime);
void CreateMbdPhysicalSystemObjects(ChSystemParallelNSC& mphysicalSystem,
                                    fsi::ChSystemFsi& myFsiSystem,
                                    chrono::fsi::SimParams* paramsH);

//------------------------------------------------------------------
// Print the simulation parameters: those pre-set and those set from
// command line
//------------------------------------------------------------------
void printSimulationParameters(chrono::fsi::SimParams* paramsH) {
    simParams << " time_pause_fluid_external_force: " << paramsH->timePause << endl
              << " contact_recovery_speed: " << contact_recovery_speed << endl
              << " maxFlowVelocity " << paramsH->v_Max << endl
              << " time_step (paramsH->dT): " << paramsH->dT << endl
              << " time_end: " << paramsH->tFinal << endl;
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    time_t rawtime;
    struct tm* timeinfo;

    // You can set your cuda device, if you have multiple of them
    // If not this should be set to 0 or not specified at all
    ////cudaSetDevice(0);

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        cout << "Error creating directory " << out_dir << endl;
        return 1;
    }

    if (ChFileutils::MakeDirectory(demo_dir.c_str()) < 0) {
        cout << "Error creating directory " << demo_dir << endl;
        return 1;
    }

    //****************************************************************************************
    const std::string simulationParams = out_dir + "/simulation_specific_parameters.txt";
    simParams.open(simulationParams);
    simParams << " Job was submitted at date/time: " << asctime(timeinfo) << endl;
    simParams.close();
    //****************************************************************************************
    bool mHaveFluid = false;
#if haveFluid
    mHaveFluid = true;
#endif
    ChSystemParallelNSC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(&mphysicalSystem, mHaveFluid);

    chrono::fsi::SimParams* paramsH = myFsiSystem.GetSimParams();

    SetupParamsH(paramsH, bxDim, byDim, bzDim, fxDim, fyDim, fzDim);
    printSimulationParameters(paramsH);
#if haveFluid

    // ********************** Create Fluid region *************************
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    utils::GridSampler<> sampler(initSpace0);
    // Use a chrono sampler to create a bucket of fluid
    chrono::fsi::Real3 boxCenter =
        chrono::fsi::mR3(-bxDim / 2 + fxDim / 2, 0 * paramsH->HSML, fzDim / 2 + 1 * paramsH->HSML);
    chrono::fsi::Real3 boxHalfDim = chrono::fsi::mR3(fxDim / 2, fyDim / 2 + 3 * paramsH->HSML, fzDim / 2);
    utils::Generator::PointVector points = sampler.SampleBox(fsi::ChFsiTypeConvert::Real3ToChVector(boxCenter),
                                                             fsi::ChFsiTypeConvert::Real3ToChVector(boxHalfDim));
    // Add from the sampler points to the FSI system
    int numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.GetDataManager()->AddSphMarker(
            chrono::fsi::mR3(points[i].x(), points[i].y(), points[i].z()), chrono::fsi::mR3(0),
            chrono::fsi::mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1));
    }

    int numPhases = myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray.size();

    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
        std::cin.get();
        return -1;
    } else {
        myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray.push_back(mI4(0, numPart, -1, -1));
        myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray.push_back(mI4(numPart, numPart, 0, 0));
    }
#endif

    // ********************** Create Rigid ******************************
    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);
    // This needs to be called after fluid initialization because we are using
    // "numObjects.numBoundaryMarkers" inside it

    // Create the objects of the Chsystem (Solid phases) using something like this function
    CreateMbdPhysicalSystemObjects(mphysicalSystem, myFsiSystem, paramsH);

    // You must call the finalize before you start the simulation
    myFsiSystem.Finalize();

    printf("\n\n sphMarkersH end%d, numAllMarkers is %d \n\n\n",
           myFsiSystem.GetDataManager()->sphMarkersH.posRadH.size(),
           myFsiSystem.GetDataManager()->numObjects.numAllMarkers);

    if (myFsiSystem.GetDataManager()->sphMarkersH.posRadH.size() !=
        myFsiSystem.GetDataManager()->numObjects.numAllMarkers) {
        printf("\n\n\n\n Error! (2) numObjects is not set correctly \n %d, %d \n\n\n",
               myFsiSystem.GetDataManager()->sphMarkersH.posRadH.size(),
               myFsiSystem.GetDataManager()->numObjects.numAllMarkers);
        return -1;
    }

    cout << " -- ChSystem size : " << mphysicalSystem.Get_bodylist().size() << endl;

    // ******************* System Initialize********************************
    // myFsiSystem.InitializeChronoGraphics(CameraLocation, CameraLookAt);

    double mTime = 0;

#ifdef CHRONO_FSI_USE_DOUBLE
    printf("Double Precision\n");
#else
    printf("Single Precision\n");
#endif
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    stepEnd = 1000000;
    std::vector<std::vector<double>> vCoor;
    std::vector<std::vector<int>> faces;

    if (save_output)
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, 0, mTime);

    const std::string rmCmd = (std::string("rm ") + demo_dir + std::string("/*"));
    system(rmCmd.c_str());

    Real time = 0;
    Real Global_max_dT = paramsH->dT;

    int next_frame = 1;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("step : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / out_fps;
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;

        if (std::abs(time - (double)next_frame * frame_time) < 1e-5 && save_output) {
            double time = paramsH->dT * tStep;
            SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);
            next_frame = next_frame + 1;
        }
    }

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------

void CreateMbdPhysicalSystemObjects(ChSystemParallelNSC& mphysicalSystem,
                                    fsi::ChSystemFsi& myFsiSystem,
                                    chrono::fsi::SimParams* paramsH) {
    std::shared_ptr<ChMaterialSurfaceNSC> mat_g(new ChMaterialSurfaceNSC);
    // Set common material Properties
    mat_g->SetFriction(0.8);
    mat_g->SetCohesion(0);
    mat_g->SetCompliance(0.0);
    mat_g->SetComplianceT(0.0);
    mat_g->SetDampingF(0.2);

    // Ground body
    auto ground = std::make_shared<ChBody>(std::make_shared<collision::ChCollisionModelParallel>());
    ground->SetIdentifier(-1);
    ground->SetBodyFixed(true);
    ground->SetCollide(true);
    ground->SetMaterialSurface(mat_g);
    ground->GetCollisionModel()->ClearModel();

    // Create the geometry of the boundaries
    // Bottom wall
    ChVector<> sizeBottom(bxDim / 2 + 3 * paramsH->HSML, byDim / 2 + 3 * paramsH->HSML, 2 * paramsH->HSML);
    ChVector<> posBottom(0, 0, -2 * paramsH->HSML);
    ChVector<> posTop(0, 0, bzDim + 2 * paramsH->HSML);

    // left and right Wall p:positive n:negative
    ChVector<> size_YZ(2 * paramsH->HSML, byDim / 2 + 3 * paramsH->HSML, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + paramsH->HSML, 0.0, bzDim / 2 + 1 * paramsH->HSML);
    ChVector<> pos_xn(-bxDim / 2 - 3 * paramsH->HSML, 0.0, bzDim / 2 + 1 * paramsH->HSML);

    // Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * paramsH->HSML, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + paramsH->HSML, bzDim / 2 + 1 * paramsH->HSML);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * paramsH->HSML, bzDim / 2 + 1 * paramsH->HSML);

    // Add the Boundaries to the chrono system
    chrono::utils::AddBoxGeometry(ground.get(), sizeBottom, posBottom, chrono::QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), size_YZ, pos_xp, chrono::QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), size_YZ, pos_xn, chrono::QUNIT, true);

    // You may uncomment the following lines to have side walls as well.
    // To show the use of Periodic boundary condition, these walls are not added
    // To this end, paramsH->cMin and paramsH->cMax were set up appropriately (see the .h file)
    // chrono::utils::AddBoxGeometry(ground.get(), size_XZ, pos_yp, chrono::QUNIT, true);
    // chrono::utils::AddBoxGeometry(ground.get(), size_XZ, pos_yn, chrono::QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

#if haveFluid
    // Add the boundaries to the FSI system
    chrono::fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posBottom, chrono::QUNIT, sizeBottom);
    chrono::fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posTop, chrono::QUNIT, sizeBottom);
    chrono::fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_xp, chrono::QUNIT, size_YZ, 23);
    chrono::fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_xn, chrono::QUNIT, size_YZ, 23);
// If you uncommented the above lines that add the side walls, you should uncomment the following two lines as well
// This is necessary in order to populate the walls with BCE markers for the fluid simulation
// chrono::fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yp, chrono::QUNIT, size_XZ, 13);
// chrono::fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yn, chrono::QUNIT, size_XZ, 13);

#endif
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemParallelNSC& mphysicalSystem,
                          chrono::fsi::SimParams* paramsH,
                          int next_frame,
                          double mTime) {
    static double exec_time;
    int out_steps = std::ceil((1.0 / paramsH->dT) / out_fps);
    exec_time += mphysicalSystem.GetTimerStep();
    static int out_frame = 0;
    chrono::fsi::utils::PrintToFile(myFsiSystem.GetDataManager()->sphMarkersD2.posRadD,
                                    myFsiSystem.GetDataManager()->sphMarkersD2.velMasD,
                                    myFsiSystem.GetDataManager()->sphMarkersD2.rhoPresMuD,
                                    myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray, demo_dir, true);
    cout << "\n------------ Output frame:   " << next_frame << endl;
    cout << "             Execution time: " << exec_time << endl;
    out_frame++;
}
