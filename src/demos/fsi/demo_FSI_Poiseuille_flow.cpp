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
// Author: Milad Rakhsha
// =============================================================================

// General Includes
#include <assert.h>
#include <stdlib.h>
#include <ctime>

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

//----------------------------
// output directories and settings
//----------------------------
const std::string out_dir = GetChronoOutputPath() + "FSI_POISEUILLE_FLOW/";
std::string demo_dir;

bool save_output = true;

Real bxDim = 0.2;
Real byDim = 0.1;
Real bzDim = 0.2;

Real fxDim = bxDim;
Real fyDim = byDim;
Real fzDim = bzDim;

void ShowUsage() {
    cout << "usage: ./demo_FSI_ <json_file>" << endl;
}
/// Forward declaration of a helper function
void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int next_frame,
                          double mTime);
//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
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
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom wall
    ChVector<> sizeBottom(bxDim / 2, byDim / 2 + 0 * initSpace0, 2 * initSpace0);
    ChVector<> posBottom(0, 0, -3 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 1 * initSpace0);

    // left and right Wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);

    // Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, sizeBottom, posBottom, chrono::QUNIT, true);
    ////chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xp, chrono::QUNIT, true);
    ////chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_YZ, pos_xn, chrono::QUNIT, true);
    ////chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yp, chrono::QUNIT, true);
    ////chrono::utils::AddBoxGeometry(ground.get(), mysurfmaterial, size_XZ, pos_yn, chrono::QUNIT, true);
    ground->GetCollisionModel()->BuildModel();
    mphysicalSystem.AddBody(ground);

    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posBottom, chrono::QUNIT, sizeBottom);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, posTop, chrono::QUNIT, sizeBottom);
    ////fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yp, chrono::QUNIT, size_XZ, 13);
    ////fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, ground, pos_yn, chrono::QUNIT, size_XZ, 13);
}

// =============================================================================

int main(int argc, char* argv[]) {
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(mphysicalSystem);
    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();
    // Use the default input file or you may enter your input parameters as a command line argument
    std::string input_json = "fsi/input_json/demo_FSI_Poiseuille_flow_I2SPH.json";
    if (argc > 1) {
        input_json = std::string(argv[1]);
    }
    std::string inputJson = GetChronoDataFile(input_json);
    if (!fsi::utils::ParseJSON(inputJson, paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
        ShowUsage();
        return 1;
    }

    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);
    // periodic boundary condition
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = fsi::mR3(-bxDim / 2 - initSpace0 / 2, -byDim / 2 - initSpace0 / 2, 0.0 - 5.0 * initSpace0);
    paramsH->cMax = fsi::mR3(bxDim / 2 + initSpace0 / 2, byDim / 2 + initSpace0 / 2, bzDim + 5.0 * initSpace0);
    // call FinalizeDomain to setup the binning for neighbor search or write your own
    fsi::utils::FinalizeDomain(paramsH);
    fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson);
    // ******************************* Create Fluid region ****************************************
    utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter1(-bxDim / 2 + fxDim / 2, 0, fzDim * 0.5);
    ChVector<> boxHalfDim1(fxDim / 2, fyDim / 2, fzDim / 2);
    utils::Generator::PointVector points = sampler.SampleBox(boxCenter1, boxHalfDim1);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.GetDataManager()->AddSphMarker(fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
                                                   fsi::mR3(1e-10),
                                                   fsi::mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1));
    }

    myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, (int)numPart, -1, -1));

    // ********************** Create Rigid ******************************
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    myFsiSystem.Finalize();

    double mTime = 0;

    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    stepEnd = 1000000;

    SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, 0, mTime);

    Real time = 0;
    Real Global_max_dT = paramsH->dT_Max;
    bool isAdaptive = false;
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

        printf("next_frame is:%d,  max dt is set to %f\n", next_frame, paramsH->dT_Max);

        if (tStep < 3 && paramsH->Adaptive_time_stepping) {
            paramsH->Adaptive_time_stepping = false;
            isAdaptive = true;
        }
        myFsiSystem.DoStepDynamics_FSI();
        paramsH->Adaptive_time_stepping = isAdaptive;
        time += paramsH->dT;
        SaveParaViewFilesMBD(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);
    }

    return 0;
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFilesMBD(fsi::ChSystemFsi& myFsiSystem,
                          ChSystemSMC& mphysicalSystem,
                          std::shared_ptr<fsi::SimParams> paramsH,
                          int next_frame,
                          double mTime) {
    static double exec_time;
    int out_steps = (int)ceil((1.0 / paramsH->dT) / paramsH->out_fps);
    exec_time += mphysicalSystem.GetTimerStep();
    int num_contacts = mphysicalSystem.GetNcontacts();
    double frame_time = 1.0 / paramsH->out_fps;
    static int out_frame = 0;

    if (save_output && std::abs(mTime - (next_frame)*frame_time) < 0.00001) {
        fsi::utils::PrintToFile(
            myFsiSystem.GetDataManager()->sphMarkersD2->posRadD, myFsiSystem.GetDataManager()->sphMarkersD2->velMasD,
            myFsiSystem.GetDataManager()->sphMarkersD2->rhoPresMuD,
            myFsiSystem.GetDataManager()->fsiGeneralData->sr_tau_I_mu_i,
            myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray, thrust::host_vector<int4>(), demo_dir, true);

        if (next_frame / out_steps == 0) {
            const std::string rmCmd = std::string("rm ") + demo_dir + std::string("/*.dat");
            system(rmCmd.c_str());
        }

        cout << "\n------------ Output frame:   " << next_frame << endl;
        cout << "             Sim frame:      " << next_frame << endl;

        out_frame++;
    }
}
