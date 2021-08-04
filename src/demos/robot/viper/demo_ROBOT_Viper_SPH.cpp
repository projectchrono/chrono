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
// Author: Wei Hu, Jason Zhou
// Chrono::FSI demo to show usage of viper rover models on SPH granular terrain
// This demo uses a plug-in viper rover model from chrono::models
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <ctime>

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkDistance.h"

#include "chrono_fsi/utils/ChUtilsTypeConvert.h"
#include "chrono_fsi/ChSystemFsi.h"
#include "chrono_fsi/utils/ChUtilsGeneratorFsi.h"
#include "chrono_fsi/utils/ChUtilsJSON.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

#define AddBoundaries

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;
using namespace chrono::viper;

using std::cout;
using std::endl;

//----------------------------
// output directories and settings
//----------------------------
const std::string out_dir = GetChronoOutputPath() + "FSI_Viper/";
std::string demo_dir;
bool pv_output = true;
bool save_obj = false;  // if true, save as Wavefront OBJ; if false, save as VTK

Real smalldis = 1.0e-9;
/// Dimension of the space domain
Real bxDim = 0.0 + smalldis;
Real byDim = 0.0 + smalldis;
Real bzDim = 0.0 + smalldis;
/// Dimension of the fluid domain
Real fxDim = 0.0 + smalldis;
Real fyDim = 0.0 + smalldis;
Real fzDim = 0.0 + smalldis;

int motor_type;           // 1 means constant rotation speed, 2 means constant torque
double motor_F;           // if motor_type==1, this means the rotation speed, if motor_type==2, this means the torque
double obstacle_density;  // density of the obstacle

// Pointer to store the viper instance
std::shared_ptr<Viper> rover;

std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
    float cr = 0.2f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
/// Forward declaration of helper functions
void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemNSC& mphysicalSystem,
                       std::shared_ptr<fsi::SimParams> paramsH,
                       int tStep,
                       double mTime);

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
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------

void CreateSolidPhase(ChSystemNSC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH);
void ShowUsage() {
    cout << "usage: ./demo_FSI_Granular_Viper <json_file>" << endl;
}

// =============================================================================
int main(int argc, char* argv[]) {
    // create a physics system
    ChSystemNSC mphysicalSystem;
    fsi::ChSystemFsi myFsiSystem(mphysicalSystem);
    // Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");
    if (argc == 1 && fsi::utils::ParseJSON(inputJson.c_str(), paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
    } else if (argc == 2 && fsi::utils::ParseJSON(argv[1], paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
    } else {
        ShowUsage();
        return 1;
    }

    /// Dimension of the space domain
    bxDim = paramsH->boxDimX + smalldis;
    byDim = paramsH->boxDimY + smalldis;
    bzDim = paramsH->boxDimZ + smalldis;
    /// Dimension of the fluid domain
    fxDim = paramsH->fluidDimX + smalldis;
    fyDim = paramsH->fluidDimY + smalldis;
    fzDim = paramsH->fluidDimZ + smalldis;

    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);
    myFsiSystem.SetFluidSystemLinearSolver(paramsH->LinearSolver);

    // fsi::utils::ParseJSON sets default values to cMin and cMax which may need
    // to be modified depending on the case (e.g periodic BC)
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    paramsH->cMin = fsi::mR3(-bxDim / 2, -byDim / 2, -bzDim - 10 * initSpace0) * 10 - 0 * initSpace0;
    paramsH->cMax = fsi::mR3(bxDim / 2, byDim / 2, bzDim + 10 * initSpace0) * 10 + 0 * initSpace0;
    // call FinalizeDomain to setup the binning for neighbor search or write your own
    fsi::utils::FinalizeDomain(paramsH);

    if (argc == 1 && fsi::utils::ParseJSON(inputJson.c_str(), paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
        fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, inputJson.c_str());
    } else if (argc == 2 && fsi::utils::ParseJSON(argv[1], paramsH, fsi::mR3(bxDim, byDim, bzDim))) {
        fsi::utils::PrepareOutputDir(paramsH, demo_dir, out_dir, argv[1]);
    }

    // ******************************* Create Fluid region ****************************************
    /// Create an initial box of fluid
    chrono::utils::GridSampler<> sampler(initSpace0);
    /// Use a chrono sampler to create a bucket of fluid
    ChVector<> boxCenter(0, 0, fzDim / 2 + 0 * initSpace0);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    chrono::utils::Generator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    /// Add fluid markers from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        Real pre_ini = paramsH->rho0 * abs(paramsH->gravity.z) * (-points[i].z() + fzDim);
        ////Real rho_ini = paramsH->rho0 + pre_ini / (paramsH->Cs * paramsH->Cs);
        myFsiSystem.GetDataManager()->AddSphMarker(
            fsi::mR4(points[i].x(), points[i].y(), points[i].z(), paramsH->HSML), fsi::mR3(0.0, 0.0, 0.0),
            fsi::mR4(paramsH->rho0, pre_ini, paramsH->mu0, -1),  // initial presssure modified as 0.0
            fsi::mR3(0.0e0),                                     // tauxxyyzz
            fsi::mR3(0.0e0));                                    // tauxyxzyz
    }

    size_t numPhases = myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.size();

    if (numPhases != 0) {
        std::cout << "Error! numPhases is wrong, thrown from main\n" << std::endl;
        std::cin.get();
        return -1;
    } else {
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(chrono::fsi::mI4(0, numPart, -1, -1));
        myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray.push_back(chrono::fsi::mI4(numPart, numPart, 0, 0));
    }

    /// Create MBD or FE model
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    /// Construction of the FSI system must be finalized
    myFsiSystem.Finalize();

    /// Get the body from the FSI system for visualization
    int stepEnd = int(paramsH->tFinal / paramsH->dT);
    stepEnd = 1000000;

    /// use the following to write a VTK file of the Rover
    std::vector<std::vector<double>> vCoor;
    std::vector<std::vector<int>> faces;
    std::string RigidConectivity = demo_dir + "RigidConectivity.vtk";

    /// Get the body from the FSI system
    std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = myFsiSystem.GetFsiBodies();
    auto Rover = FSI_Bodies[0];

    SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, 0, 0);

    /// write the Penetration into file
    std::ofstream myFile;
    myFile.open("./body_position.txt", std::ios::trunc);
    myFile.close();
    myFile.open("./body_position.txt", std::ios::app);
    myFile << 0.0 << "\t" << Rover->GetPos().x() << "\t" << Rover->GetPos().y() << "\t" << Rover->GetPos().z() << "\t"
           << Rover->GetPos_dt().x() << "\t" << Rover->GetPos_dt().y() << "\t" << Rover->GetPos_dt().z() << "\n";
    myFile.close();
    Real time = 0;
    Real Global_max_dT = paramsH->dT_Max;
    for (int tStep = 0; tStep < stepEnd + 1; tStep++) {
        printf("\nstep : %d, time= : %f (s) \n", tStep, time);
        double frame_time = 1.0 / paramsH->out_fps;
        int next_frame = (int)floor((time + 1e-6) / frame_time) + 1;
        double next_frame_time = next_frame * frame_time;
        double max_allowable_dt = next_frame_time - time;
        if (max_allowable_dt > 1e-7)
            paramsH->dT_Max = std::min(Global_max_dT, max_allowable_dt);
        else
            paramsH->dT_Max = Global_max_dT;

        rover->Update();
        myFsiSystem.DoStepDynamics_FSI();
        time += paramsH->dT;
        SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);

        auto bbody = mphysicalSystem.Get_bodylist()[0];
        auto rbody = mphysicalSystem.Get_bodylist()[1];

        printf("bin=%f,%f,%f\n", bbody->GetPos().x(), bbody->GetPos().y(), bbody->GetPos().z());
        printf("Rover=%f,%f,%f\n", rbody->GetPos().x(), rbody->GetPos().y(), rbody->GetPos().z());
        printf("Rover=%f,%f,%f\n", rbody->GetPos_dt().x(), rbody->GetPos_dt().y(), rbody->GetPos_dt().z());
        myFile.open("./body_position.txt", std::ios::app);
        myFile << time << "\t" << rbody->GetPos().x() << "\t" << rbody->GetPos().y() << "\t" << rbody->GetPos().z()
               << "\t" << rbody->GetPos_dt().x() << "\t" << rbody->GetPos_dt().y() << "\t" << rbody->GetPos_dt().z()
               << "\n";
        myFile.close();

        if (time > paramsH->tFinal)
            break;
    }

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if fsi, their
// bce representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& mphysicalSystem,
                      fsi::ChSystemFsi& myFsiSystem,
                      std::shared_ptr<fsi::SimParams> paramsH) {
    Real initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);
    mphysicalSystem.Set_G_acc(gravity);

    // Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    // mysurfmaterial->SetYoungModulus(1e8);
    // mysurfmaterial->SetFriction(0.5f);
    // mysurfmaterial->SetRestitution(0.05f);
    // mysurfmaterial->SetAdhesion(0);

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    auto mfloor = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    mfloor->SetPos(ChVector<>(0, 0, 0));
    mfloor->SetBodyFixed(true);
    mphysicalSystem.Add(mfloor);

    /// Bottom wall
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 2 * initSpace0);

    /// left and right Wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    /// Front and back Wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    /// Fluid-Solid Coupling at the walls via Condition Enforcement (BCE) Markers
    // fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, mfloor, pos_zp, QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, mfloor, pos_zn, QUNIT, size_XY, 12);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, mfloor, pos_xp, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, mfloor, pos_xn, QUNIT, size_YZ, 23);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, mfloor, pos_yp, QUNIT, size_XZ, 13);
    fsi::utils::AddBoxBce(myFsiSystem.GetDataManager(), paramsH, mfloor, pos_yn, QUNIT, size_XZ, 13);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    rover = chrono_types::make_shared<Viper>(&mphysicalSystem);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(ChVector<>(paramsH->bodyIniPosX, paramsH->bodyIniPosY, paramsH->bodyIniPosZ), QUNIT));

    // add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(WheelID::LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(WheelID::RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(WheelID::LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(WheelID::RB)->GetBody();
        }

        myFsiSystem.AddFsiBody(wheel_body);
        std::string BCE_path = GetChronoDataFile("fsi/demo_BCE/BCE_viperWheel.txt");
        fsi::utils::AddBCE_FromFile(myFsiSystem.GetDataManager(), paramsH, wheel_body, BCE_path, ChVector<double>(0),
                                    QUNIT, 1.0);
    }
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(fsi::ChSystemFsi& myFsiSystem,
                       ChSystemNSC& mphysicalSystem,
                       std::shared_ptr<fsi::SimParams> paramsH,
                       int next_frame,
                       double mTime) {
    double frame_time = 1.0 / paramsH->out_fps;
    char filename[4096];
    static int out_frame = 0;

    if (pv_output && std::abs(mTime - (next_frame)*frame_time) < 1e-7) {
        // save the SPH particles
        fsi::utils::PrintToFile(myFsiSystem.GetDataManager()->sphMarkersD2->posRadD,
                                myFsiSystem.GetDataManager()->sphMarkersD2->velMasD,
                                myFsiSystem.GetDataManager()->sphMarkersD2->rhoPresMuD,
                                myFsiSystem.GetDataManager()->fsiGeneralData->sr_tau_I_mu_i,
                                myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray,
                                myFsiSystem.GetDataManager()->fsiGeneralData->referenceArray_FEA, demo_dir, true);

        // save the viper body to obj/vtk files
        for (int i = 0; i < 1; i++) {
            auto body = rover->GetChassis()->GetBody();
            ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
            ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

            auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            std::string obj_path = (GetChronoDataFile("robot/viper/obj/viper_chassis.obj"));
            double scale_ratio = 1.0;
            mmesh->LoadWavefrontMesh(obj_path, false, true);
            mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
            mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

            double mmass;
            ChVector<> mcog;
            ChMatrix33<> minertia;
            mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
            mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

            if (save_obj) {  // save to obj file
                sprintf(filename, "%s/body_%d.obj", paramsH->demo_dir, next_frame);
                std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
                geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
            } else {
                sprintf(filename, "%s/body_%d.vtk", paramsH->demo_dir, next_frame);
                std::ofstream file;
                file.open(filename);
                file << "# vtk DataFile Version 2.0" << std::endl;
                file << "VTK from simulation" << std::endl;
                file << "ASCII" << std::endl;
                file << "DATASET UNSTRUCTURED_GRID" << std::endl;
                auto nv = mmesh->getCoordsVertices().size();
                file << "POINTS " << nv << " float" << std::endl;
                for (auto& v : mmesh->getCoordsVertices())
                    file << v.x() << " " << v.y() << " " << v.z() << std::endl;
                auto nf = mmesh->getIndicesVertexes().size();
                file << "CELLS " << nf << " " << 4 * nf << std::endl;
                for (auto& f : mmesh->getIndicesVertexes())
                    file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
                file << "CELL_TYPES " << nf << std::endl;
                for (size_t ii = 0; ii < nf; ii++)
                    file << "5 " << std::endl;
                file.close();
            }
        }

        // save the wheels to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetWheel(WheelID::LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetWheel(WheelID::RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetWheel(WheelID::LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetWheel(WheelID::RB)->GetBody();
            }

            ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
            ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

            auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            std::string obj_path = GetChronoDataFile("robot/viper/obj/viper_wheel.obj");
            double scale_ratio = 1.0;
            mmesh->LoadWavefrontMesh(obj_path, false, true);
            mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
            mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

            double mmass;
            ChVector<> mcog;
            ChMatrix33<> minertia;
            mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
            mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

            if (save_obj) {  // save to obj file
                sprintf(filename, "%s/wheel_%d_%d.obj", paramsH->demo_dir, i + 1, next_frame);
                std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
                geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
            } else {  // save to vtk file
                sprintf(filename, "%s/wheel_%d_%d.vtk", paramsH->demo_dir, i + 1, next_frame);
                std::ofstream file;
                file.open(filename);
                file << "# vtk DataFile Version 2.0" << std::endl;
                file << "VTK from simulation" << std::endl;
                file << "ASCII" << std::endl;
                file << "DATASET UNSTRUCTURED_GRID" << std::endl;
                auto nv = mmesh->getCoordsVertices().size();
                file << "POINTS " << nv << " float" << std::endl;
                for (auto& v : mmesh->getCoordsVertices())
                    file << v.x() << " " << v.y() << " " << v.z() << std::endl;
                auto nf = mmesh->getIndicesVertexes().size();
                file << "CELLS " << nf << " " << 4 * nf << std::endl;
                for (auto& f : mmesh->getIndicesVertexes())
                    file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
                file << "CELL_TYPES " << nf << std::endl;
                for (size_t ii = 0; ii < nf; ii++)
                    file << "5 " << std::endl;
                file.close();
            }
        }

        // save the steering rod to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetUpright(WheelID::LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetUpright(WheelID::RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetUpright(WheelID::LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetUpright(WheelID::RB)->GetBody();
            }
            ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
            ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

            auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            std::string obj_path = "";
            if (i == 0 || i == 2) {
                obj_path = GetChronoDataFile("robot/viper/obj/viper_L_steer.obj");
            }
            if (i == 1 || i == 3) {
                obj_path = GetChronoDataFile("robot/viper/obj/viper_R_steer.obj");
            }
            double scale_ratio = 1.0;
            mmesh->LoadWavefrontMesh(obj_path, false, true);
            mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
            mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

            double mmass;
            ChVector<> mcog;
            ChMatrix33<> minertia;
            mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
            mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

            if (save_obj) {  // save to obj file
                sprintf(filename, "%s/steerRod_%d_%d.obj", paramsH->demo_dir, i + 1, next_frame);
                std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
                geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
            } else {  // save to vtk file
                sprintf(filename, "%s/steerRod_%d_%d.vtk", paramsH->demo_dir, i + 1, next_frame);
                std::ofstream file;
                file.open(filename);
                file << "# vtk DataFile Version 2.0" << std::endl;
                file << "VTK from simulation" << std::endl;
                file << "ASCII" << std::endl;
                file << "DATASET UNSTRUCTURED_GRID" << std::endl;
                auto nv = mmesh->getCoordsVertices().size();
                file << "POINTS " << nv << " float" << std::endl;
                for (auto& v : mmesh->getCoordsVertices())
                    file << v.x() << " " << v.y() << " " << v.z() << std::endl;
                auto nf = mmesh->getIndicesVertexes().size();
                file << "CELLS " << nf << " " << 4 * nf << std::endl;
                for (auto& f : mmesh->getIndicesVertexes())
                    file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
                file << "CELL_TYPES " << nf << std::endl;
                for (size_t ii = 0; ii < nf; ii++)
                    file << "5 " << std::endl;
                file.close();
            }
        }

        // save the lower rod to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetLowerArm(WheelID::LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetLowerArm(WheelID::RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetLowerArm(WheelID::LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetLowerArm(WheelID::RB)->GetBody();
            }
            ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
            ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

            auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            std::string obj_path = "";
            if (i == 0 || i == 2) {
                obj_path = GetChronoDataFile("robot/viper/obj/viper_L_bt_sus.obj");
            }
            if (i == 1 || i == 3) {
                obj_path = GetChronoDataFile("robot/viper/obj/viper_R_bt_sus.obj");
            }
            double scale_ratio = 1.0;
            mmesh->LoadWavefrontMesh(obj_path, false, true);
            mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
            mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

            double mmass;
            ChVector<> mcog;
            ChMatrix33<> minertia;
            mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
            mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

            if (save_obj) {  // save to obj file
                sprintf(filename, "%s/lowerRod_%d_%d.obj", paramsH->demo_dir, i + 1, next_frame);
                std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
                geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
            } else {  // save to vtk file
                sprintf(filename, "%s/lowerRod_%d_%d.vtk", paramsH->demo_dir, i + 1, next_frame);
                std::ofstream file;
                file.open(filename);
                file << "# vtk DataFile Version 2.0" << std::endl;
                file << "VTK from simulation" << std::endl;
                file << "ASCII" << std::endl;
                file << "DATASET UNSTRUCTURED_GRID" << std::endl;
                auto nv = mmesh->getCoordsVertices().size();
                file << "POINTS " << nv << " float" << std::endl;
                for (auto& v : mmesh->getCoordsVertices())
                    file << v.x() << " " << v.y() << " " << v.z() << std::endl;
                auto nf = mmesh->getIndicesVertexes().size();
                file << "CELLS " << nf << " " << 4 * nf << std::endl;
                for (auto& f : mmesh->getIndicesVertexes())
                    file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
                file << "CELL_TYPES " << nf << std::endl;
                for (size_t ii = 0; ii < nf; ii++)
                    file << "5 " << std::endl;
                file.close();
            }
        }

        // save the upper rod to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetUpperArm(WheelID::LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetUpperArm(WheelID::RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetUpperArm(WheelID::LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetUpperArm(WheelID::RB)->GetBody();
            }
            ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
            ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

            auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
            std::string obj_path = "";
            if (i == 0 || i == 2) {
                obj_path = GetChronoDataFile("robot/viper/obj/viper_L_up_sus.obj");
            }
            if (i == 1 || i == 3) {
                obj_path = GetChronoDataFile("robot/viper/obj/viper_R_up_sus.obj");
            }

            double scale_ratio = 1.0;
            mmesh->LoadWavefrontMesh(obj_path, false, true);
            mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
            mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

            double mmass;
            ChVector<> mcog;
            ChMatrix33<> minertia;
            mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
            mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

            if (save_obj) {  // save to obj file
                sprintf(filename, "%s/upperRod_%d_%d.obj", paramsH->demo_dir, i + 1, next_frame);
                std::vector<geometry::ChTriangleMeshConnected> meshes = {*mmesh};
                geometry::ChTriangleMeshConnected::WriteWavefront(filename, meshes);
            } else {  // save to vtk file
                sprintf(filename, "%s/upperRod_%d_%d.vtk", paramsH->demo_dir, i + 1, next_frame);
                std::ofstream file;
                file.open(filename);
                file << "# vtk DataFile Version 2.0" << std::endl;
                file << "VTK from simulation" << std::endl;
                file << "ASCII" << std::endl;
                file << "DATASET UNSTRUCTURED_GRID" << std::endl;
                auto nv = mmesh->getCoordsVertices().size();
                file << "POINTS " << nv << " float" << std::endl;
                for (auto& v : mmesh->getCoordsVertices())
                    file << v.x() << " " << v.y() << " " << v.z() << std::endl;
                auto nf = mmesh->getIndicesVertexes().size();
                file << "CELLS " << nf << " " << 4 * nf << std::endl;
                for (auto& f : mmesh->getIndicesVertexes())
                    file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
                file << "CELL_TYPES " << nf << std::endl;
                for (size_t ii = 0; ii < nf; ii++)
                    file << "5 " << std::endl;
                file.close();
            }
        }

        // save box obstacle to vtk files
        for (int i = 0; i < 2; i++) {
            sprintf(filename, "%s/obstacle_%d_%d.vtk", paramsH->demo_dir, i + 1, next_frame);
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET POLYDATA" << std::endl;

            file << "POINTS " << 8 << " "
                 << "float" << std::endl;
            auto Body = mphysicalSystem.Get_bodylist()[i + 2 + 16];
            ChVector<> center = Body->GetPos();
            ChMatrix33<> Rotation = Body->GetRot();
            double lx = paramsH->bodyDimX * 0.5;
            double ly = paramsH->bodyDimY * 0.5;
            double lz = paramsH->bodyDimZ * 0.5;
            ChVector<double> Node1 = ChVector<double>(-lx, -ly, -lz);
            ChVector<double> Node2 = ChVector<double>(lx, -ly, -lz);
            ChVector<double> Node3 = ChVector<double>(lx, -ly, lz);
            ChVector<double> Node4 = ChVector<double>(-lx, -ly, lz);
            ChVector<double> Node5 = ChVector<double>(-lx, ly, -lz);
            ChVector<double> Node6 = ChVector<double>(lx, ly, -lz);
            ChVector<double> Node7 = ChVector<double>(lx, ly, lz);
            ChVector<double> Node8 = ChVector<double>(-lx, ly, lz);
            ChVector<double> vertex1 = Rotation * Node1 + center;
            ChVector<double> vertex2 = Rotation * Node2 + center;
            ChVector<double> vertex3 = Rotation * Node3 + center;
            ChVector<double> vertex4 = Rotation * Node4 + center;
            ChVector<double> vertex5 = Rotation * Node5 + center;
            ChVector<double> vertex6 = Rotation * Node6 + center;
            ChVector<double> vertex7 = Rotation * Node7 + center;
            ChVector<double> vertex8 = Rotation * Node8 + center;
            file << vertex1.x() << " " << vertex1.y() << " " << vertex1.z() << "\n";
            file << vertex2.x() << " " << vertex2.y() << " " << vertex2.z() << "\n";
            file << vertex3.x() << " " << vertex3.y() << " " << vertex3.z() << "\n";
            file << vertex4.x() << " " << vertex4.y() << " " << vertex4.z() << "\n";
            file << vertex5.x() << " " << vertex5.y() << " " << vertex5.z() << "\n";
            file << vertex6.x() << " " << vertex6.y() << " " << vertex6.z() << "\n";
            file << vertex7.x() << " " << vertex7.y() << " " << vertex7.z() << "\n";
            file << vertex8.x() << " " << vertex8.y() << " " << vertex8.z() << "\n";

            file << "POLYGONS " << 6 << " " << 30 << std::endl;
            file << "4 " << 0 << " " << 1 << " " << 2 << " " << 3 << "\n";
            file << "4 " << 0 << " " << 1 << " " << 5 << " " << 4 << "\n";
            file << "4 " << 0 << " " << 4 << " " << 7 << " " << 3 << "\n";
            file << "4 " << 4 << " " << 5 << " " << 6 << " " << 7 << "\n";
            file << "4 " << 1 << " " << 5 << " " << 6 << " " << 2 << "\n";
            file << "4 " << 3 << " " << 2 << " " << 6 << " " << 7 << "\n";
        }

        // save rigid body position and rotation
        for (int i = 1; i < mphysicalSystem.Get_bodylist().size(); i++) {
            auto body = mphysicalSystem.Get_bodylist()[i];
            ChFrame<> ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> pos = ref_frame.GetPos();
            ChQuaternion<> rot = ref_frame.GetRot();
            ChVector<> vel = body->GetPos_dt();

            std::string delim = ",";
            sprintf(filename, "%s/body_pos_rot_vel%d.csv", paramsH->demo_dir, i);
            std::ofstream file;
            if (mphysicalSystem.GetChTime() > 0)
                file.open(filename, std::fstream::app);
            else {
                file.open(filename);
                file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1"
                     << delim << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << std::endl;
            }

            file << mphysicalSystem.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim
                 << rot.e0() << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim
                 << vel.y() << delim << vel.z() << std::endl;

            file.close();
        }

        cout << "-------------------------------------\n" << endl;
        cout << "             Output frame:   " << next_frame << endl;
        cout << "             Time:           " << mTime << endl;
        cout << "-------------------------------------\n" << endl;

        out_frame++;
    }
}