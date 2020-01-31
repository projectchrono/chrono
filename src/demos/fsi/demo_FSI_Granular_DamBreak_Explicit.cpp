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
// Author: Wei Hu, Milad Rakhsha
// =============================================================================

// General Includes
#include <assert.h>
#include <limits.h>
#include <stdlib.h>
#include <thrust/extrema.h>
#include <time.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"

// Chrono fsi includes
//#include "chrono_fsi/ChUtilsDevice.cuh"
#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJSON.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

#define ElasticSPH  // only for modelling elastic and granular problems

// Chrono namespaces
using namespace chrono;
using namespace collision;

using std::cout;
using std::endl;
std::ofstream simParams;

//----------------------------
// output directories and settings
//----------------------------
const std::string out_dir = GetChronoOutputPath() + "FSI_DamBreak/";
std::string demo_dir;
// Save data as csv files, turn it on to be able to see the results off-line using paraview
bool save_output = true;
// Frequency of the save output
typedef fsi::Real Real;

Real smalldis = 1.0e-9;
// Dimension of the space domain
Real bxDim = 0.0 + smalldis;
Real byDim = 0.0 + smalldis;
Real bzDim = 0.0 + smalldis;
/// Dimension of the fluid domain
Real fxDim = 0.0 + smalldis;
Real fyDim = 0.0 + smalldis;
Real fzDim = 0.0 + smalldis;

// Forward declaration of helper functions
void Calculator(fsi::ChSystemFsi& myFsiSystem, double time);
void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int tStep,
                          double mTime);
void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH);
void ShowUsage() {
    cout << "usage: ./demo_FSI_Granular_DamBreak_Explicit <json_file>" << endl;
}

// ******** Main function of the demo ********* //
int main(int argc, char* argv[]) {
    // create a physics system
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(mphysicalSystem);
    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();
    std::string input_json = "fsi/input_json/demo_FSI_Granular_DamBreak_Explicit.json";

    // create an fsi system and choose integrator type: I2SPH, ExplicitSPH
    std::string inputJson = GetChronoDataFile(input_json);
    if (!fsi::utils::ParseJSON(inputJson, paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
        ShowUsage();
        return 1;
    }

    // Dimension of the space domain
    bxDim = paramsH->boxDimX + smalldis;
    byDim = paramsH->boxDimY + smalldis;
    bzDim = paramsH->boxDimZ + smalldis;
    // Dimension of the fluid domain
    fxDim = paramsH->fluidDimX + smalldis;
    fyDim = paramsH->fluidDimY + smalldis;
    fzDim = paramsH->fluidDimZ + smalldis;

    // set the Linear Solver (might not need in ExplicitSPH)
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);

    // fsi::utils::ParseJSON sets default values to cMin and cMax which may need
    // to be modified depending on the case (e.g periodic BC)
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin =
        fsi::mR3(-bxDim / 2 - 3.0 * initSpace0, -byDim / 2 - 1.0 * initSpace0 / 2.0, -bzDim / 2 - 3.0 * initSpace0);
    paramsH->cMax =
        fsi::mR3(bxDim / 2 + 3.0 * initSpace0, byDim / 2 + 1.0 * initSpace0 / 2.0, bzDim / 2 + 3.0 * initSpace0);

    // call FinalizeDomain to setup the binning for neighbor search or write your own
    fsi::utils::FinalizeDomain(paramsH);
    fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);

    // choose a GPU
    cudaSetDevice(0);

    // ******************************* Create Fluid region ****************************************
    utils::GridSampler<> sampler(initSpace0);
    // Use a chrono sampler to create a bucket of fluid
    ChVector<> boxCenter(-bxDim / 2 + fxDim / 2, 0.0, -bzDim / 2 + fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    // Add fluid markers from the sampler points to the FSI system
    int numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        Real vz = 0.0;
        // if(points[i].x()>-1.25){
        //     vz = (points[i].x() + 1.25)/3.0*0.25;
        // }
        Real pre_ini = paramsH->rho0 * abs(paramsH->gravity.z) * (-points[i].z() - bzDim / 2 + fzDim);
        Real rho_ini = paramsH->rho0 + pre_ini / (paramsH->Cs * paramsH->Cs);
        myFsiSystem.GetDataManager()->AddSphMarker(
            fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML), fsi::mR3(0.0, 0.0, vz),
            fsi::mR4(rho_ini, pre_ini, paramsH->mu0, -1),  // initial presssure modified as 0.0
            fsi::mR3(0.0e0),                               // tauxxyyzz
            fsi::mR3(0.0e0));                              // tauxyxzyz
    }

    int numPhases = myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.size();

    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
        std::cin.get();
        return -1;
    } else {
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, numPart, -1, -1));
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(numPart, numPart, 0, 0));
    }
    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);

    /// Create the objects of the Chsystem (Solid phases) using something like this function
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    /// You must call the finalize before you start the simulation
    myFsiSystem.Finalize();

    double mTime = 0;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    stepEnd = 1e8;
    // std::vector<std::vector<double>> vCoor;
    // std::vector<std::vector<int>> faces;

    /// save the particle information before the simulation
    if (save_output) {
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, 0, mTime);
    }

    Real time = 0;
    Real Global_max_dT = paramsH->dT_Max;
    double TIMING = clock();

    // start the simulation
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int next_frame = std::floor((time + 1e-6) / frame_time) + 1;
        double next_frame_time = next_frame * frame_time;
        double max_allowable_dt = next_frame_time - time;
        if (max_allowable_dt > 1e-6)
            paramsH->dT_Max = std::min(Global_max_dT, max_allowable_dt);
        else
            paramsH->dT_Max = Global_max_dT;

        printf("next_frame is:%d,  max dt is set to %f\n", next_frame, paramsH->dT_Max);
        myFsiSystem.DoStepDynamics_FSI();

        time += paramsH->dT;
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);
        Calculator(myFsiSystem, time);
        if (time > paramsH->tFinal)
            break;
    }

    double test = (clock() - TIMING) / (double)CLOCKS_PER_SEC;
    printf("Finished in %f\n", test);

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------

void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH) {
    std::shared_ptr<ChMaterialSurfaceNSC> mat_g(new ChMaterialSurfaceNSC);
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
    ground->SetMaterialSurface(mysurfmaterial);
    ground->GetCollisionModel()->ClearModel();

    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    Real smalldis = 1.0e-9;

    /// Create the geometry of the boundaries
    /// Bottom and Top wall
    Real Lwall = 0.25;  // length of the fixed wall at the end of the beam
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2, 2 * initSpace0);
    ChVector<> posBottom(0.0, 0.0, -bzDim / 2 - 3 * initSpace0);
    ChVector<> posTop(0.0, 0.0, bzDim / 2 + 1 * initSpace0);

    /// left and right Wall p:positive n:negative
    ChVector<> size_YZ(2 * initSpace0, byDim / 2, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + 1 * initSpace0, 0.0, 0.0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, 0.0);

    /// Front and back Wall
    // ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    // ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    // ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    /// Add the Boundaries to the chrono system
    chrono::utils::AddBoxGeometry(ground.get(), size_XY, posBottom, QUNIT, true);
    // chrono::utils::AddBoxGeometry(ground.get(), size_XY, posTop, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), size_YZ, pos_xp, QUNIT, true);
    chrono::utils::AddBoxGeometry(ground.get(), size_YZ, pos_xn, QUNIT, true);

    /// You may uncomment the following lines to have side walls as well.
    /// To show the use of Periodic boundary condition, these walls are not added
    /// To this end, paramsH->cMin and paramsH->cMax were set up appropriately (see the .h file)
    // chrono::utils::AddBoxGeometry(ground.get(), size_XZ, pos_yp, QUNIT, true);
    // chrono::utils::AddBoxGeometry(ground.get(), size_XZ, pos_yn, QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

    // Add the boundaries to the FSI system
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posBottom, QUNIT, size_XY);
    // fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posTop, QUNIT, size_XY);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_xp, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_xn, QUNIT, size_YZ, 23);
    // If you uncommented the above lines that add the side walls, you should uncomment the following two lines as well
    // This is necessary in order to populate the walls with BCE markers for the fluid simulation
    // fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yp, QUNIT, size_XZ, 13);
    // fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yn, QUNIT, size_XZ, 13);
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int next_frame,
                          double mTime) {
    static double exec_time;
    int out_steps = std::ceil((1.0 / paramsH->dT) / paramsH->out_fps);
    static int out_frame = 0;
    double frame_time = 1.0 / paramsH->out_fps;
    if (save_output && std::abs(mTime - (next_frame)*frame_time) < 1e-7) {
        fsi::utils::PrintToFile(
            myFsiSystem.GetDataManager()->sphMarkersD2->posRadD, myFsiSystem.GetDataManager()->sphMarkersD2->velMasD,
            myFsiSystem.GetDataManager()->sphMarkersD2->rhoPresMuD,
            myFsiSystem.GetDataManager()->fsiGeneralData->sr_tau_I_mu_i,
            myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray, thrust::host_vector<int4>(), demo_dir, true);
        cout << "-------------------------------------\n" << endl;
        cout << "             Output frame:   " << next_frame << endl;
        cout << "             Time:           " << mTime << endl;
        cout << "-------------------------------------\n" << endl;
        out_frame++;
    }
}

void Calculator(fsi::ChSystemFsi& myFsiSystem, double time) {
    std::ofstream output;
    output.open((out_dir + "/Analysis.txt").c_str(), std::ios::app);
    thrust::host_vector<int4> referenceArray = myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray;
    fsi::ChUtilsDevice fsiUtils;
    fsiUtils.CopyD2H(myFsiSystem.GetDataManager()->sphMarkersD2->posRadD,
                     myFsiSystem.GetDataManager()->sphMarkersH->posRadH);
    thrust::host_vector<fsi::Real4> posRad = myFsiSystem.GetDataManager()->sphMarkersH->posRadH;
    struct compare_Real4_x {
        __host__ __device__ bool operator()(fsi::Real4 lhs, fsi::Real4 rhs) { return lhs.x < rhs.x; }
    };
    struct compare_Real4_y {
        __host__ __device__ bool operator()(fsi::Real4 lhs, fsi::Real4 rhs) { return lhs.z < rhs.z; }
    };
    thrust::host_vector<fsi::Real4>::iterator iter =
        thrust::max_element(posRad.begin(), posRad.begin() + referenceArray[0].y, compare_Real4_x());
    double MaxPosx = (*iter).x;
    iter = thrust::max_element(posRad.begin(), posRad.begin() + referenceArray[0].y, compare_Real4_y());
    double MaxPosy = (*iter).z;
    output << time << " " << MaxPosx << " " << MaxPosy << " " << std::endl;
    output.close();
}
