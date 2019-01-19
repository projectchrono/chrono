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
#include <cassert>
#include <climits>
#include <cstdlib>
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

// Chrono general utils
#include "chrono/core/ChTransform.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_thirdparty/filesystem/resolver.h"

// Chrono fsi includes
#include "chrono_fsi/ChDeviceUtils.cuh"
#include "chrono_fsi/ChFsiTypeConvert.h"
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJsonInput.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

#define AddBoundaries

// Chrono namespaces
using namespace chrono;
using namespace collision;
using namespace filesystem;

using std::cout;
using std::endl;
std::ofstream simParams;
//****************************************************************************************
// Output directory
const std::string out_dir = GetChronoOutputPath() + "FSI_CYLINDER_DROP/";
std::string demo_dir;
bool pv_output = true;
typedef fsi::Real Real;

/// Dimensions of the cylinder, fluid and boundary
Real bxDim = 1;
Real byDim = 0.55;
Real bzDim = 1.3;

Real fxDim = bxDim;
Real fyDim = byDim;
Real fzDim = 1;

double cyl_length = 0.15001;
double cyl_radius = .12;
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/// Forward declaration of helper functions
void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemSMC& mphysicalSystem,
                       fsi::SimParams* paramsH,
                       int tStep,
                       double mTime,
                       std::shared_ptr<ChBody> Cylinder);

void AddWall(std::shared_ptr<ChBody> body, const ChVector<>& dim, const ChVector<>& loc) {
    body->GetCollisionModel()->AddBox(dim.x(), dim.y(), dim.z(), loc);
    auto box = std::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = dim;
    box->GetBoxGeometry().Pos = loc;
}

void ShowUsage() {
    cout << "usage: ./demo_FSI_CylinderDrop <json_file>" << endl;
    cout << "or to use default input parameters ./demo_FSI_CylinderDrop " << endl;
}
//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& mphysicalSystem, fsi::ChSystemFsi& myFsiSystem, fsi::SimParams* paramsH) {
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);
    mphysicalSystem.Set_G_acc(gravity);

    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
    /// Set common material Properties
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.2f);
    mysurfmaterial->SetRestitution(0.05);
    mysurfmaterial->SetAdhesion(0);

    /// Bottom wall
    ChVector<> sizeBottom(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> posBottom(0, 0, -2 * initSpace0);
    ChVector<> posTop(0, 0, bzDim + 2 * initSpace0);

    /// left and right Wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 1 * initSpace0);

    /// Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 1 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 1 * initSpace0);

    /// Create a container
    auto bin = std::make_shared<ChBody>(ChMaterialSurface::SMC);
    bin->SetPos(ChVector<>(0.0, 0.0, 0.0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetIdentifier(-1);
    bin->SetBodyFixed(true);
    bin->GetCollisionModel()->ClearModel();
    bin->GetCollisionModel()->SetSafeMargin(initSpace0 / 2);
    bin->SetMaterialSurface(mysurfmaterial);
    /// MBD representation of the walls
    AddWall(bin, sizeBottom, posBottom);
    AddWall(bin, sizeBottom, posTop + ChVector<>(0.0, 0.0, 3 * initSpace0));
    AddWall(bin, size_YZ, pos_xp);
    AddWall(bin, size_YZ, pos_xn);
    AddWall(bin, size_XZ, pos_yp + ChVector<>(+1.5 * initSpace0, +1.5 * initSpace0, 0.0));
    AddWall(bin, size_XZ, pos_yn + ChVector<>(-0.5 * initSpace0, -0.5 * initSpace0, 0.0));
    bin->GetCollisionModel()->BuildModel();

    bin->SetCollide(true);
    mphysicalSystem.AddBody(bin);

    /// Fluid-Solid Coupling at the walls via Condition Enforcement (BCE) Markers
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, posBottom, QUNIT, sizeBottom);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, posTop, QUNIT, sizeBottom);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_xp, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_xn, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_yp, QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_yn, QUNIT, size_XZ, 13);

    /// Create falling cylinder
    ChVector<> cyl_pos = ChVector<>(0, 0, fzDim + cyl_radius + 2 * initSpace0);
    ChQuaternion<> cyl_rot = QUNIT;
    auto cylinder = std::make_shared<ChBody>(ChMaterialSurface::SMC);
    cylinder->SetPos(cyl_pos);
    double volume = utils::CalcCylinderVolume(cyl_radius, cyl_length);
    ChVector<> gyration = utils::CalcCylinderGyration(cyl_radius, cyl_length).Get_Diag();

    // This is the interesting part, sanity check suing the Archimedes' principle
    // If you reduce the density of the solid you should be able to see
    // it does not submerge. Alternatively, if you increase this density you
    // will see that the cylinder fully sinks.
    // If water density is used, then the cylinder should float
    double density = 2 * paramsH->rho0;
    double mass = density * volume;
    cylinder->SetCollide(true);
    cylinder->SetBodyFixed(false);
    cylinder->SetMass(mass);
    cylinder->SetMaterialSurface(mysurfmaterial);
    cylinder->SetInertiaXX(mass * gyration);
    cylinder->GetCollisionModel()->ClearModel();
    cylinder->GetCollisionModel()->SetSafeMargin(initSpace0);
    utils::AddCylinderGeometry(cylinder.get(), cyl_radius, cyl_length, ChVector<>(0.0, 0.0, 0.0),
                               ChQuaternion<>(1, 0, 0, 0));
    cylinder->GetCollisionModel()->BuildModel();
    int numRigidObjects = mphysicalSystem.Get_bodylist().size();
    mphysicalSystem.AddBody(cylinder);

    /// Add this body to the FSI system
    myFsiSystem.AddFsiBody(cylinder);
    /// Fluid-Solid Coupling of the cylinder via Condition Enforcement (BCE) Markers
    fsi::utils::AddCylinderBce(myFsiSystem.GetDataManager(), paramsH, cylinder, ChVector<>(0, 0, 0),
                               ChQuaternion<>(1, 0, 0, 0), cyl_radius, cyl_length + initSpace0, paramsH->HSML, false);
}

// =============================================================================

int main(int argc, char* argv[]) {
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(&mphysicalSystem, true, fsi::ChFluidDynamics::Integrator::IISPH);
    fsi::SimParams* paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_CylinderDrop.json");
    if (argc == 1 && fsi::utils::ParseJSON(inputJson.c_str(), paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
    } else if (argc == 2 && fsi::utils::ParseJSON(argv[1], paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
    } else {
        ShowUsage();
        return 1;
    }

    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = fsi::mR3(-bxDim / 2, -byDim / 2, -bzDim / 2 - 5 * initSpace0) * 10 - 4 * initSpace0;
    paramsH->cMax = fsi::mR3(bxDim / 2, byDim / 2, bzDim + 10 * initSpace0) * 10 + 4 * initSpace0;
    // call FinalizeDomainCreating to setup the binning for neighbor search or write your own
    fsi::utils::FinalizeDomainCreating(paramsH);

    if (strcmp(paramsH->out_name, "Undefined") == 0)
        demo_dir = out_dir + "Paraview/";
    else
        demo_dir = out_dir + paramsH->out_name + "/";

    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(demo_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    const std::string removeFiles = (std::string("rm ") + demo_dir + "/* ");
    system(removeFiles.c_str());
    if (argc == 2) {
        const std::string copyInputs = (std::string("cp ") + argv[1] + " " + demo_dir);
        system(copyInputs.c_str());
    }

    /// Create an initial box of fluid
    utils::GridSampler<> sampler(initSpace0);
    ChVector<> boxCenter(0, 0 * initSpace0, fzDim / 2 + 1 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    int numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        myFsiSystem.GetDataManager()->AddSphMarker(fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
                                                   fsi::mR3(1e-10),
                                                   fsi::mR4(paramsH->rho0, paramsH->BASEPRES, paramsH->mu0, -1));
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

    /// Create MBD or FE model
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);
    /// Construction of the FSI system must be finalized
    myFsiSystem.Finalize();

    /// Get the cylinder body from the FSI system for visualization
    double mTime = 0;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    stepEnd = 1000000;

    /// use the following to write a VTK file of the cylinder
    std::vector<std::vector<double>> vCoor;
    std::vector<std::vector<int>> faces;
    std::string RigidConectivity = demo_dir + "RigidConectivity.vtk";

    /// Set up integrator for the MBD
    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(mphysicalSystem.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-6);
    mystepper->SetMode(ChTimestepperHHT::ACCELERATION);
    mystepper->SetScaling(true);

    /// Get the body from the FSI system
    std::vector<std::shared_ptr<ChBody>>* FSI_Bodies = (myFsiSystem.GetFsiBodiesPtr());
    auto Cylinder = ((*FSI_Bodies)[0]);
    SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, 0, 0, Cylinder);

    Real time = 0;
    Real Global_max_dT = paramsH->dT_Max;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int next_frame = std::floor((time + 1e-6) / frame_time) + 1;
        double next_frame_time = next_frame * frame_time;
        double max_allowable_dt = next_frame_time - time;
        if (max_allowable_dt > 1e-7)
            paramsH->dT_Max = std::min(Global_max_dT, max_allowable_dt);
        else
            paramsH->dT_Max = Global_max_dT;

        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
        SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, next_frame, time, Cylinder);

        auto bin = mphysicalSystem.Get_bodylist()[0];
        auto cyl = mphysicalSystem.Get_bodylist()[1];

        printf("bin=%f,%f,%f\n", bin->GetPos().x(), bin->GetPos().y(), bin->GetPos().z());
        printf("cyl=%f,%f,%f\n", cyl->GetPos().x(), cyl->GetPos().y(), cyl->GetPos().z());

        if (time > 10.0)
            break;
    }

    return 0;
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemSMC& mphysicalSystem,
                       fsi::SimParams* paramsH,
                       int next_frame,
                       double mTime,
                       std::shared_ptr<ChBody> Cylinder) {
    int out_steps = std::ceil((1.0 / paramsH->dT) / paramsH->out_fps);
    int num_contacts = mphysicalSystem.GetNcontacts();
    double frame_time = 1.0 / paramsH->out_fps;
    static int out_frame = 0;

    if (pv_output && std::abs(mTime - (next_frame)*frame_time) < 1e-7) {
        fsi::utils::PrintToFile(myFsiSystem.GetDataManager()->sphMarkersD2.posRadD,
                                myFsiSystem.GetDataManager()->sphMarkersD2.velMasD,
                                myFsiSystem.GetDataManager()->sphMarkersD2.rhoPresMuD,
                                myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray,
                                myFsiSystem.GetDataManager()->fsiGeneralData.referenceArray_FEA, demo_dir, true);
        cout << "-------------------------------------\n" << endl;
        cout << "             Output frame:   " << next_frame << endl;
        cout << "             Time:           " << mTime << endl;
        cout << "-------------------------------------\n" << endl;

        out_frame++;
    }
}
