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
// Chrono::FSI demo to show usage of VIPER rover models on SPH granular terrain
// This demo uses a plug-in VIPER rover model from chrono::models
// =============================================================================

/// General Includes
#include <cassert>
#include <cstdlib>
#include <ctime>

/// Chrono includes
#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChParticleCloud.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkDistance.h"

/// Chrono fsi includes
#include "chrono_fsi/ChSystemFsi.h"

/// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::geometry;
using namespace chrono::viper;

/// output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_VIPER/";
std::string demo_dir;
bool pv_output = true;
bool save_obj = false;  // if true, save as Wavefront OBJ; if false, save as VTK

double smalldis = 1.0e-9;

/// Dimension of the space domain
double bxDim = 1.0 + smalldis;
double byDim = 1.0 + smalldis;
double bzDim = 0.2 + smalldis;

/// Dimension of the terrain domain
double fxDim = 1.0 + smalldis;
double fyDim = 1.0 + smalldis;
double fzDim = 0.1 + smalldis;

/// Pointer to store the VIPER instance
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

/// Forward declaration of helper functions
void SaveParaViewFiles(ChSystemFsi& myFsiSystem,
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
}

void CreateSolidPhase(ChSystemNSC& mphysicalSystem, ChSystemFsi& myFsiSystem, std::shared_ptr<fsi::SimParams> paramsH);

void ShowUsage() {
    std::cout << "usage: ./demo_FSI_Granular_Viper <json_file>" << std::endl;
}

int main(int argc, char* argv[]) {
    /// Create a physical system and a corresponding FSI system
    ChSystemNSC mphysicalSystem;
    ChSystemFsi myFsiSystem(mphysicalSystem);

    /// Get the pointer to the system parameter and use a JSON file to fill it out with the user parameters
    std::shared_ptr<fsi::SimParams> paramsH = myFsiSystem.GetSimParams();
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");
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

    /// Set SPH discretization type, consistent or inconsistent
    myFsiSystem.SetDiscreType(false, false);

    /// Set wall boundary condition
    myFsiSystem.SetWallBC(BceVersion::ORIGINAL);

    /// Reset the domain size
    bxDim = paramsH->boxDimX + smalldis;
    byDim = paramsH->boxDimY + smalldis;
    bzDim = paramsH->boxDimZ + smalldis;

    fxDim = paramsH->fluidDimX + smalldis;
    fyDim = paramsH->fluidDimY + smalldis;
    fzDim = paramsH->fluidDimZ + smalldis;

    /// Setup the solver based on the input value of the prameters
    myFsiSystem.SetFluidDynamics(paramsH->fluid_dynamic_type);

    /// Set the periodic boundary condition
    double initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;
    ChVector<> cMin(-bxDim / 2 * 2, -byDim / 2 * 2, -bzDim * 10);
    ChVector<> cMax(bxDim / 2 * 2, byDim / 2 * 2, bzDim * 10);
    myFsiSystem.SetBoundaries(cMin, cMax, paramsH);

    /// Setup sub doamins for a faster neighbor particle searching
    myFsiSystem.SetSubDomain(paramsH);

    /// Setup the output directory for FSI data
    myFsiSystem.SetFsiOutputDir(paramsH, demo_dir, out_dir, inputJson.c_str());

    /// Set FSI information output
    myFsiSystem.SetFsiInfoOutput(false);

    /// Set simulation data output length
    myFsiSystem.SetOutputLength(0);

    /// Create an initial box for the terrain patch
    chrono::utils::GridSampler<> sampler(initSpace0);
    /// Use a chrono sampler to create a bucket of granular material
    ChVector<> boxCenter(0, 0, fzDim / 2);
    ChVector<> boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2);
    std::vector<ChVector<>> points = sampler.SampleBox(boxCenter, boxHalfDim);
    /// Add SPH particles from the sampler points to the FSI system
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = paramsH->rho0 * abs(paramsH->gravity.z) * (-points[i].z() + fzDim);
        myFsiSystem.AddSphMarker(points[i], paramsH->rho0, 0, paramsH->mu0, paramsH->HSML, -1,
                                 ChVector<>(0),         // initial velocity
                                 ChVector<>(-pre_ini),  // tauxxyyzz
                                 ChVector<>(0)          // tauxyxzyz
        );
    }
    myFsiSystem.AddRefArray(0, (int)numPart, -1, -1);

    /// Create MBD and BCE particles for the solid domain
    CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);

    /// Construction of the FSI system must be finalized
    myFsiSystem.Finalize();

    /// Get the body from the FSI system for visualization
    std::vector<std::shared_ptr<ChBody>>& FSI_Bodies = myFsiSystem.GetFsiBodies();
    auto Rover = FSI_Bodies[0];

    /// Save data at the initial moment
    SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, 0, 0);

    /// write the Penetration into file
    std::ofstream myFile;
    myFile.open("./body_position.txt", std::ios::trunc);
    myFile.close();
    myFile.open("./body_position.txt", std::ios::app);
    myFile << 0.0 << "\t" << Rover->GetPos().x() << "\t" << Rover->GetPos().y() << "\t" << Rover->GetPos().z() << "\t"
           << Rover->GetPos_dt().x() << "\t" << Rover->GetPos_dt().y() << "\t" << Rover->GetPos_dt().z() << "\n";
    myFile.close();
    double time = 0;
    double Global_max_dT = paramsH->dT_Max;
    int stepEnd = int(paramsH->tFinal / paramsH->dT);

    /// Add timing for ths simulation
    double TIMING_sta;
    double TIMING_end;
    double sim_cost = 0.0;

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

        TIMING_sta = clock();
        myFsiSystem.DoStepDynamics_FSI();
        TIMING_end = clock();
        sim_cost = sim_cost + (TIMING_end - TIMING_sta) / (double)CLOCKS_PER_SEC;

        time += paramsH->dT;
        SaveParaViewFiles(myFsiSystem, mphysicalSystem, paramsH, next_frame, time);

        auto bbody = mphysicalSystem.Get_bodylist()[0];
        auto rbody = mphysicalSystem.Get_bodylist()[1];

        printf("bin=%f,%f,%f\n", bbody->GetPos().x(), bbody->GetPos().y(), bbody->GetPos().z());
        printf("Rover=%f,%f,%f\n", rbody->GetPos().x(), rbody->GetPos().y(), rbody->GetPos().z());
        printf("Rover=%f,%f,%f\n", rbody->GetPos_dt().x(), rbody->GetPos_dt().y(), rbody->GetPos_dt().z());
        printf("Physical time and computational cost = %f, %f\n", time, sim_cost);
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
// Create the objects of the MBD system. Rigid bodies, and if fsi,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemNSC& mphysicalSystem, ChSystemFsi& myFsiSystem, std::shared_ptr<fsi::SimParams> paramsH) {
    /// Set the gravity force for the simulation
    ChVector<> gravity = ChVector<>(paramsH->gravity.x, paramsH->gravity.y, paramsH->gravity.z);
    mphysicalSystem.Set_G_acc(gravity);

    /// Set common material Properties
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0025);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.0025);

    /// Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    box->SetPos(ChVector<>(0, 0, 0));
    box->SetBodyFixed(true);
    mphysicalSystem.Add(box);

    /// Get the initial SPH particle spacing
    double initSpace0 = paramsH->MULT_INITSPACE * paramsH->HSML;

    /// Bottom wall
    ChVector<> size_XY(bxDim / 2 + 3 * initSpace0, byDim / 2 + 3 * initSpace0, 2 * initSpace0);
    ChVector<> pos_zn(0, 0, -3 * initSpace0);
    ChVector<> pos_zp(0, 0, bzDim + 2 * initSpace0);

    /// Left and right wall
    ChVector<> size_YZ(2 * initSpace0, byDim / 2 + 3 * initSpace0, bzDim / 2);
    ChVector<> pos_xp(bxDim / 2 + initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_xn(-bxDim / 2 - 3 * initSpace0, 0.0, bzDim / 2 + 0 * initSpace0);

    /// Front and back wall
    ChVector<> size_XZ(bxDim / 2, 2 * initSpace0, bzDim / 2);
    ChVector<> pos_yp(0, byDim / 2 + initSpace0, bzDim / 2 + 0 * initSpace0);
    ChVector<> pos_yn(0, -byDim / 2 - 3 * initSpace0, bzDim / 2 + 0 * initSpace0);

    /// Fluid-Solid Coupling at the walls via BCE particles
    myFsiSystem.AddBceBox(paramsH, box, pos_zn, QUNIT, size_XY, 12);
    myFsiSystem.AddBceBox(paramsH, box, pos_xp, QUNIT, size_YZ, 23);
    myFsiSystem.AddBceBox(paramsH, box, pos_xn, QUNIT, size_YZ, 23);
    myFsiSystem.AddBceBox(paramsH, box, pos_yp, QUNIT, size_XZ, 13);
    myFsiSystem.AddBceBox(paramsH, box, pos_yn, QUNIT, size_XZ, 13);

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    rover = chrono_types::make_shared<Viper>(&mphysicalSystem);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(ChVector<>(paramsH->bodyIniPosX, paramsH->bodyIniPosY, paramsH->bodyIniPosZ), QUNIT));

    /// Add BCE particles and mesh of wheels to the system
    for (int i = 0; i < 4; i++) {
        std::shared_ptr<ChBodyAuxRef> wheel_body;
        if (i == 0) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
        }
        if (i == 1) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
        }
        if (i == 2) {
            wheel_body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
        }
        if (i == 3) {
            wheel_body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
        }

        myFsiSystem.AddFsiBody(wheel_body);
        std::string BCE_path = GetChronoDataFile("fsi/demo_BCE/BCE_viperWheel.txt");
        if (i == 0 || i == 2) {
            myFsiSystem.AddBceFile(paramsH, wheel_body, BCE_path, ChVector<>(0), Q_from_AngZ(CH_C_PI), 1.0, true);
        } else {
            myFsiSystem.AddBceFile(paramsH, wheel_body, BCE_path, ChVector<>(0), QUNIT, 1.0, true);
        }
    }
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(ChSystemFsi& myFsiSystem,
                       ChSystemNSC& mphysicalSystem,
                       std::shared_ptr<fsi::SimParams> paramsH,
                       int next_frame,
                       double mTime) {
    double frame_time = 1.0 / paramsH->out_fps;
    char filename[4096];

    if (pv_output && std::abs(mTime - (next_frame)*frame_time) < 1e-5) {
        /// save the SPH particles
        myFsiSystem.PrintParticleToFile(demo_dir);

        /// save the VIPER body to obj/vtk files
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
            } else {  // save to vtk file
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

        /// save the wheels to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetWheel(ViperWheelID::V_LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetWheel(ViperWheelID::V_RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetWheel(ViperWheelID::V_LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetWheel(ViperWheelID::V_RB)->GetBody();
            }

            ChFrame<> body_ref_frame = body->GetFrame_REF_to_abs();
            ChVector<> body_pos = body_ref_frame.GetPos();      // body->GetPos();
            ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();
            if (i == 0 || i == 2) {
                body_rot.Cross(body_rot, Q_from_AngZ(CH_C_PI));
            }

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

        /// save the steering rod to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetUpright(ViperWheelID::V_LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetUpright(ViperWheelID::V_RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetUpright(ViperWheelID::V_LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetUpright(ViperWheelID::V_RB)->GetBody();
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

        /// save the lower rod to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetLowerArm(ViperWheelID::V_LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetLowerArm(ViperWheelID::V_RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetLowerArm(ViperWheelID::V_LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetLowerArm(ViperWheelID::V_RB)->GetBody();
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

        /// save the upper rod to obj/vtk files
        for (int i = 0; i < 4; i++) {
            std::shared_ptr<ChBodyAuxRef> body;
            if (i == 0) {
                body = rover->GetUpperArm(ViperWheelID::V_LF)->GetBody();
            }
            if (i == 1) {
                body = rover->GetUpperArm(ViperWheelID::V_RF)->GetBody();
            }
            if (i == 2) {
                body = rover->GetUpperArm(ViperWheelID::V_LB)->GetBody();
            }
            if (i == 3) {
                body = rover->GetUpperArm(ViperWheelID::V_RB)->GetBody();
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

        /// save box obstacle to vtk files
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

        /// save rigid body position and rotation
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

        std::cout << "-------------------------------------\n" << std::endl;
        std::cout << "             Output frame:   " << next_frame << std::endl;
        std::cout << "             Time:           " << mTime << std::endl;
        std::cout << "-------------------------------------\n" << std::endl;
    }
}
