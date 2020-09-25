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
#include <cassert>
#include <cstdlib>
#include <ctime>

// Chrono includes
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/core/ChTransform.h"

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
const std::string out_dir = GetChronoOutputPath() + "FSI_CYLINDER_DROP/";
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

// Size of the cylinder
double cyl_length;
double cyl_radius;

// -----------------------------------------------------------------
void ShowUsage() {
    cout << "usage: ./demo_FSI_CylinderDrop <json_file>" << endl;
    cout << "or to use default input parameters ./demo_FSI_CylinderDrop " << endl;
}

//------------------------------------------------------------------
// Function to add walls into Chrono system
//------------------------------------------------------------------
void AddWall(std::shared_ptr<ChBody> body, 
             const ChVector<>& dim, 
             std::shared_ptr<ChMaterialSurface> mat,
             const ChVector<>& loc) {
    body->GetCollisionModel()->AddBox(mat, dim.x(), dim.y(), dim.z(), loc);
    auto box = chrono_types::make_shared<ChBoxShape>();
    box->GetBoxGeometry().Size = dim;
    box->GetBoxGeometry().Pos = loc;
}

//------------------------------------------------------------------
// Function to save cylinder to Paraview VTK files
//------------------------------------------------------------------
void WriteCylinderVTK(std::shared_ptr<ChBody> Body, 
                      double radius, 
                      double length, 
                      int res, 
                      char SaveAsBuffer[256]) {
    std::ofstream output;
    output.open(SaveAsBuffer, std::ios::app);
    output << "# vtk DataFile Version 1.0\nUnstructured Grid Example\nASCII\n\n" << std::endl;
    output << "DATASET UNSTRUCTURED_GRID\nPOINTS " << 2 * res << " float\n";

    ChVector<> center = Body->GetPos();
    // printf("POS=%f,%f,%f", center.x(), center.y(), center.z());

    ChMatrix33<> Rotation = Body->GetRot();
    ChVector<double> vertex;
    for (int i = 0; i < res; i++) {
        ChVector<double> thisNode;
        thisNode.x() = radius * cos(2 * i * 3.1415 / res);
        thisNode.y() = -1 * length / 2;
        thisNode.z() = radius * sin(2 * i * 3.1415 / res);
        vertex = Rotation * thisNode + center;  // rotate/scale, if needed
        output << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
    }

    for (int i = 0; i < res; i++) {
        ChVector<double> thisNode;
        thisNode.x() = radius * cos(2 * i * 3.1415 / res);
        thisNode.y() = +1 * length / 2;
        thisNode.z() = radius * sin(2 * i * 3.1415 / res);
        vertex = Rotation * thisNode + center;  // rotate/scale, if needed
        output << vertex.x() << " " << vertex.y() << " " << vertex.z() << "\n";
    }

    output << "\n\nCELLS " << (unsigned int)res + res << "\t" << (unsigned int)5 * (res + res) << "\n";

    for (int i = 0; i < res - 1; i++) {
        output << "4 " << i << " " << i + 1 << " " << i + res + 1 << " " << i + res << "\n";
    }
    output << "4 " << res - 1 << " " << 0 << " " << res << " " << 2 * res - 1 << "\n";

    for (int i = 0; i < res / 4; i++) {
        output << "4 " << i << " " << i + 1 << " " << +res / 2 - i - 1 << " " << +res / 2 - i << "\n";
    }

    for (int i = 0; i < res / 4; i++) {
        output << "4 " << i + res << " " << i + 1 + res << " " << +res / 2 - i - 1 + res << " " << +res / 2 - i + res
               << "\n";
    }

    output << "4 " << +res / 2 << " " << 1 + res / 2 << " " << +res - 1 << " " << 0 << "\n";

    for (int i = 1; i < res / 4; i++) {
        output << "4 " << i + res / 2 << " " << i + 1 + res / 2 << " " << +res / 2 - i - 1 + res / 2 << " "
               << +res / 2 - i + res / 2 << "\n";
    }

    output << "4 " << 3 * res / 2 << " " << 1 + 3 * res / 2 << " " << +2 * res - 1 << " " << +res << "\n";

    for (int i = 1; i < res / 4; i++) {
        output << "4 " << i + 3 * res / 2 << " " << i + 1 + 3 * res / 2 << " " << +2 * res - i - 1 << " "
               << +2 * res - i << "\n";
    }

    output << "\nCELL_TYPES " << res + res << "\n";

    for (int iele = 0; iele < (res + res); iele++) {
        output << "9\n";
    }
}

//------------------------------------------------------------------
// Function to save the paraview files
//------------------------------------------------------------------
void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemSMC& mphysicalSystem,
                       std::shared_ptr<fsi::SimParams> paramsH,
                       int this_frame,
                       double mTime,
                       std::shared_ptr<ChBody> Cylinder) {
    // Simulation steps between two output frames
    int out_steps = (int)ceil((1.0 / paramsH->dT) / paramsH->out_fps);

    // Simulation time between two output frames
    double frame_time = 1.0 / paramsH->out_fps;

    // Output data to files
    if (save_output && std::abs(mTime - (this_frame)*frame_time) < 1e-9) {
        // save particles to cvs files
        fsi::utils::PrintToFile(myFsiSystem.GetDataManager()->sphMarkersD2->posRadD,
                                myFsiSystem.GetDataManager()->sphMarkersD2->velMasD,
                                myFsiSystem.GetDataManager()->sphMarkersD2->rhoPresMuD,
                                myFsiSystem.GetDataManager()->fsiGeneralData->sr_tau_I_mu_i,
                                myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray,
                                myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray_FEA, demo_dir, true);
        // save rigid bodies to vtk files
        char SaveAsRigidObjVTK[256];
        static int RigidCounter = 0;
        snprintf(SaveAsRigidObjVTK, sizeof(char) * 256, (demo_dir + "/Cylinder.%d.vtk").c_str(), RigidCounter);
        WriteCylinderVTK(Cylinder, cyl_radius, cyl_length, 100, SaveAsRigidObjVTK);
        RigidCounter++;
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
    // Set gravity to the rigid body system in chrono
    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);
    mphysicalSystem.Set_G_acc(gravity);

    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(1e8);
    mysurfmaterial->SetFriction(0.2f);
    mysurfmaterial->SetRestitution(0.05f);
    mysurfmaterial->SetAdhesion(0);

    // Create the geometry of the boundaries
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    // Bottom and Top wall - size and position
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 1 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);

    // Left and right Wall - size and position
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    // Front and back Wall - size and position
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    // Create a container
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetPos(ChVector<>(0.0, 0.0, 0.0));
    bin->SetRot(ChQuaternion<>(1, 0, 0, 0));
    bin->SetIdentifier(-1);
    bin->SetBodyFixed(true);
    bin->GetCollisionModel()->ClearModel();
    bin->GetCollisionModel()->SetSafeMargin(initSpace0 / 2);

    // Add the walls into chrono system
    AddWall(bin, size_XY, mysurfmaterial, pos_zp);
    AddWall(bin, size_XY, mysurfmaterial, pos_zn);
    AddWall(bin, size_YZ, mysurfmaterial, pos_xp);
    AddWall(bin, size_YZ, mysurfmaterial, pos_xn);
    AddWall(bin, size_XZ, mysurfmaterial, pos_yp);
    AddWall(bin, size_XZ, mysurfmaterial, pos_yn);
    bin->GetCollisionModel()->BuildModel();
    bin->SetCollide(true);
    mphysicalSystem.AddBody(bin);

    // Add BCE particles attached on the walls into FSI system
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_zp, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_zn, chrono::QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_xp, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_xn, chrono::QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_yp, chrono::QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, bin, pos_yn, chrono::QUNIT, size_XZ, 13);

    // Create a falling cylinder
    auto cylinder = chrono_types::make_shared<ChBody>();

    // Set the general properties of the cylinder
    double volume = utils::CalcCylinderVolume(cyl_radius, cyl_length / 2);
    double density = paramsH->rho0 * 2.0;
    double mass = density * volume;
    ChVector<> cyl_pos = ChVector<>(0, 0, fzDim + cyl_radius + 2 * initSpace0);
    ChVector<> cyl_vel = ChVector<>(0.0, 0.0, 0.0);
    ChQuaternion<> cyl_rot = QUNIT;
    ChVector<> gyration = utils::CalcCylinderGyration(cyl_radius, cyl_length / 2).diagonal();
    cylinder->SetPos(cyl_pos);
    cylinder->SetPos_dt(cyl_vel);
    cylinder->SetMass(mass);
    cylinder->SetInertiaXX(mass * gyration);

    // Set the collision type of the cylinder
    cylinder->SetCollide(true);
    cylinder->SetBodyFixed(false);
    cylinder->GetCollisionModel()->ClearModel();
    cylinder->GetCollisionModel()->SetSafeMargin(initSpace0);
    utils::AddCylinderGeometry(cylinder.get(), mysurfmaterial, cyl_radius, cyl_length, 
                               ChVector<>(0.0, 0.0, 0.0),
                               ChQuaternion<>(1, 0, 0, 0));
    cylinder->GetCollisionModel()->BuildModel();

    // Add this body to chrono system
    mphysicalSystem.AddBody(cylinder);

    // Add this body to the FSI system (only those have inetraction with fluid)
    myFsiSystem.AddFsiBody(cylinder);

    // Add BCE particles attached on the cylinder into FSI system
    fsi::utils::AddCylinderBce(myFsiSystem.GetDataManager(), paramsH, cylinder, ChVector<>(0, 0, 0),
                               ChQuaternion<>(1, 0, 0, 0), cyl_radius, cyl_length + initSpace0, paramsH->HSML, false);
}


// =============================================================================
int main(int argc, char* argv[]) {
     // Create a physics system and an FSI system
    ChSystemSMC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(mphysicalSystem);

    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_CylinderDrop_Explicit.json");
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

    // Size of the dropping cylinder
    cyl_radius = paramsH->bodyRad;
    cyl_length = paramsH->bodyLength;

    // Set up the periodic boundary condition (if not, set relative larger values)
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = chrono::fsi::mR3(-bxDim / 2, -byDim / 2, -bzDim - 10 * initSpace0) * 10;
    paramsH->cMax = chrono::fsi::mR3( bxDim / 2,  byDim / 2,  bzDim + 10 * initSpace0) * 10;

    // Set the time integration type and the linear solver type (only for ISPH)
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);

    // Call FinalizeDomainCreating to setup the binning for neighbor search
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
        // Calculate the pressure of a steady state (p = rho*g*h)
        Real pre_ini = paramsH->rho0 * abs(paramsH->gravity.z) * (-points[i].z() + fzDim);
        Real rho_ini = paramsH->rho0 + pre_ini / (paramsH->Cs * paramsH->Cs);
        myFsiSystem.GetDataManager()->AddSphMarker(
            fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML),
            fsi::mR3(1e-10),
            fsi::mR4(rho_ini, pre_ini, paramsH->mu0, -1));
    }
    size_t numPhases = myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.size();
    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
        std::cin.get();
        return -1;
    } else {
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4(0, (int)numPart, -1, -1));
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(mI4((int)numPart, (int)numPart, 0, 0));
    }

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    // Construction of the FSI system must be finalized before running
    myFsiSystem.Finalize();

    // Set up integrator for the multi-body dynamics system
    mphysicalSystem.SetTimestepperType(ChTimestepper::Type::HHT);
    auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(mphysicalSystem.GetTimestepper());
    mystepper->SetAlpha(-0.2);
    mystepper->SetMaxiters(1000);
    mystepper->SetAbsTolerances(1e-6);
    mystepper->SetMode(ChTimestepperHHT::ACCELERATION);
    mystepper->SetScaling(true);

    // Start the simulation
    Real time = 0;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    double TIMING_sta = clock();
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int this_frame = (int)floor((time + 1e-9) / frame_time);

        // Get the position of the container and cylinder
        auto bin = mphysicalSystem.Get_bodylist()[0];
        auto cyl = mphysicalSystem.Get_bodylist()[1];

        printf("bin=%f,%f,%f\n", bin->GetPos().x(), bin->GetPos().y(), bin->GetPos().z());
        printf("cyl=%f,%f,%f\n", cyl->GetPos().x(), cyl->GetPos().y(), cyl->GetPos().z());

        // Get the cylinder from the FSI system and Save data of the simulation
        std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = myFsiSystem.GetFsiBodies();
        auto Cylinder = FSI_Bodies[0];
        SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, this_frame, time, Cylinder);
        
        // Call the FSI solver
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
    }

    // Total computational cost
    double TIMING_end = (clock() - TIMING_sta) / (double)CLOCKS_PER_SEC;
    printf("\nSimulation Finished in %f (s)\n", TIMING_end);

    return 0;
}

