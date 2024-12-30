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
// Chrono::FSI demo to show usage of VIPER rover models on CRM granular terrain
// This demo uses a plug-in VIPER rover model from chrono::models
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fsi/sph/ChFsiSystemSPH.h"

#include "chrono_fsi/sph/visualization/ChFsiVisualization.h"
#ifdef CHRONO_OPENGL
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationGL.h"
#endif
#ifdef CHRONO_VSG
    #include "chrono_fsi/sph/visualization/ChFsiVisualizationVSG.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fsi;
using namespace chrono::viper;

// Run-time visualization system (OpenGL or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Viper/";

// If true, save as Wavefront OBJ; if false, save as VTK
bool save_obj = false;

// Physical properties of terrain particles
double initSpacing = 0.01;
double kernelMultiplier = 1;
double density = 1700.0;

// Dimension of the space domain
double bxDim = 4.0;
double byDim = 2.0;
double bzDim = 0.1;

// Rover initial location
ChVector3d init_loc(-bxDim / 2.0 + 1.0, 0, bzDim + 0.3);

// Simulation time and stepsize
double total_time = 20.0;
double dT = 2.5e-4;

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int out_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// Pointer to store the VIPER instance
std::shared_ptr<Viper> rover;

// Define Viper rover wheel type
ViperWheelType wheel_type = ViperWheelType::RealWheel;

// Use below mesh file if the wheel type is real VIPER wheel
std::string wheel_obj = "robot/viper/obj/viper_simplewheel.obj";

std::shared_ptr<ChContactMaterial> CustomWheelMaterial(ChContactMethod contact_method) {
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
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
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
            return std::shared_ptr<ChContactMaterial>();
    }
}

// Forward declaration of helper functions
void SaveParaViewFiles(double time, ChFsiSystemSPH& sysFSI);
void CreateSolidPhase(ChFsiSystemSPH& sysFSI);

int main(int argc, char* argv[]) {
    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/particles"))) {
        std::cerr << "Error creating directory " << out_dir + "/particles" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/fsi"))) {
        std::cerr << "Error creating directory " << out_dir + "/fsi" << std::endl;
        return 1;
    }
    if (!filesystem::create_directory(filesystem::path(out_dir + "/rover"))) {
        std::cerr << "Error creating directory " << out_dir + "/rover" << std::endl;
        return 1;
    }

    // Create a physical system and a corresponding FSI system
    ChSystemNSC sysMBS;
    ChFluidSystemSPH sysSPH;
    ChFsiSystemSPH sysFSI(sysMBS, sysSPH);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    // Set the simulation stepsize
    sysFSI.SetStepSizeCFD(dT);
    sysFSI.SetStepsizeMBD(dT);

    // Read JSON file with simulation parameters
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Viper_granular_NSC.json");
    if (argc == 2) {
        inputJson = std::string(argv[1]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_ROBOT_Viper_SPH <json_file>" << std::endl;
        return 1;
    }
    sysSPH.ReadParametersFromFile(inputJson);

    // Set the initial particle spacing
    sysSPH.SetInitialSpacing(initSpacing);

    // Set the SPH kernel (h = kernelMultiplier * initSpacing)
    sysSPH.SetKernelMultiplier(kernelMultiplier);

    // Set the terrain density
    sysSPH.SetDensity(density);

    // Set the simulation domain size
    sysSPH.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysSPH.SetConsistentDerivativeDiscretization(false, false);

    // Set the SPH method
    sysSPH.SetSPHMethod(SPHMethod::WCSPH);

    // Set the periodic boundary condition
    double initSpace0 = sysSPH.GetInitialSpacing();
    ChVector3d cMin(-bxDim / 2 * 2, -byDim / 2 - 0.5 * initSpacing, -bzDim * 10);
    ChVector3d cMax(bxDim / 2 * 2, byDim / 2 + 0.5 * initSpacing, bzDim * 20);
    sysSPH.SetBoundaries(cMin, cMax);

    // Set simulation data output level
    sysSPH.SetOutputLevel(OutputLevel::STATE);

    // Create an initial box for the terrain patch
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(0, 0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2, byDim / 2, bzDim / 2);
    std::vector<ChVector3d> points = sampler.SampleBox(boxCenter, boxHalfDim);

    // Add SPH particles from the sampler points to the FSI system
    std::cout << "Generate SPH particles (" << points.size() << ")" << std::endl;
    auto gz = std::abs(gravity.z());
    int numPart = (int)points.size();
    for (int i = 0; i < numPart; i++) {
        double pre_ini = sysSPH.GetDensity() * gz * (-points[i].z() + bzDim);
        sysSPH.AddSPHParticle(points[i], sysSPH.GetDensity(), 0, sysSPH.GetViscosity(),
                              ChVector3d(0),         // initial velocity
                              ChVector3d(-pre_ini),  // tauxxyyzz
                              ChVector3d(0)          // tauxyxzyz
        );
    }

    // Create MBD and BCE particles for the solid domain
    std::cout << "Generate BCE markers" << std::endl;
    CreateSolidPhase(sysFSI);

    // Complete construction of the FSI system
    sysFSI.Initialize();

    // Write position and velocity to file
    std::ofstream ofile;
    if (output)
        ofile.open(out_dir + "./body_position.txt");

        // Create run-time visualization
#ifndef CHRONO_OPENGL
    if (vis_type == ChVisualSystem::Type::OpenGL)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::OpenGL;
#endif

#if !defined(CHRONO_OPENGL) && !defined(CHRONO_VSG)
    render = false;
#endif

    std::shared_ptr<ChFsiVisualization> visFSI;
    if (render) {
        switch (vis_type) {
            case ChVisualSystem::Type::OpenGL:
#ifdef CHRONO_OPENGL
                visFSI = chrono_types::make_shared<ChFsiVisualizationGL>(&sysFSI);
#endif
                break;
            case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
                visFSI = chrono_types::make_shared<ChFsiVisualizationVSG>(&sysFSI);
#endif
                break;
            }
        }

        visFSI->SetTitle("Viper on CRM terrain");
        visFSI->SetSize(1280, 720);
        visFSI->AddCamera(ChVector3d(0, -3 * byDim, bzDim), ChVector3d(0, 0, 0));
        visFSI->SetCameraMoveScale(1.0f);
        visFSI->EnableBoundaryMarkers(true);
        visFSI->EnableRigidBodyMarkers(false);
        visFSI->SetRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetParticleRenderMode(ChFsiVisualization::RenderMode::SOLID);
        visFSI->SetSPHColorCallback(
            chrono_types::make_shared<ParticleHeightColorCallback>(ChColor(0.10f, 0.40f, 0.65f), 0, bzDim));
        visFSI->AttachSystem(&sysMBS);
        visFSI->Initialize();
    }

    // Start the simulation
    unsigned int output_steps = (unsigned int)round(1 / (out_fps * dT));
    unsigned int render_steps = (unsigned int)round(1 / (render_fps * dT));
    double time = 0.0;
    int current_step = 0;

    auto body = sysMBS.GetBodies()[1];

    ChTimer timer;
    while (time < total_time) {
        std::cout << current_step << "  time: " << time << "  sim. time: " << timer() << std::endl;

        rover->Update();

        std::cout << "  pos: " << body->GetPos() << std::endl;
        std::cout << "  vel: " << body->GetPosDt() << std::endl;
        if (output) {
            ofile << time << "  " << body->GetPos() << "    " << body->GetPosDt() << std::endl;
            if (current_step % output_steps == 0) {
                sysSPH.SaveParticleData(out_dir + "/particles");
                sysSPH.SaveSolidData(out_dir + "/fsi", time);
                SaveParaViewFiles(time, sysFSI);
            }
        }

        // Render system
        if (render && current_step % render_steps == 0) {
            if (!visFSI->Render())
                break;
        }

        timer.start();
        sysFSI.DoStepDynamics(dT);
        timer.stop();

        time += dT;
        current_step++;
    }

    if (output)
        ofile.close();

    return 0;
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies and their
// BCE representations are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChFsiSystemSPH& sysFSI) {
    ChFluidSystemSPH& sysSPH = sysFSI.GetFluidSystemSPH();
    ChSystem& sysMBS = sysFSI.GetMultibodySystem();

    // Create a body for the rigid soil container
    auto box = chrono_types::make_shared<ChBodyEasyBox>(10, 10, 0.02, 1000, false, false);
    box->SetPos(ChVector3d(0, 0, 0));
    box->SetFixed(true);
    sysMBS.Add(box);

    // Get the initial SPH particle spacing
    double initSpace0 = sysSPH.GetInitialSpacing();

    // Fluid-Solid Coupling at the walls via BCE particles
    sysSPH.AddBoxContainerBCE(box,                                        //
                              ChFrame<>(ChVector3d(0, 0, bzDim), QUNIT),  //
                              ChVector3d(bxDim, byDim, 2 * bzDim),        //
                              ChVector3i(2, 0, -1));

    auto driver = chrono_types::make_shared<ViperDCMotorControl>();
    rover = chrono_types::make_shared<Viper>(&sysMBS, wheel_type);
    rover->SetDriver(driver);
    rover->SetWheelContactMaterial(CustomWheelMaterial(ChContactMethod::NSC));
    rover->Initialize(ChFrame<>(init_loc, QUNIT));

    // Create the wheel's BCE particles
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    std::vector<ChVector3d> BCE_wheel;
    sysSPH.CreatePoints_Mesh(*trimesh, initSpace0, BCE_wheel);

    // Add BCE particles and mesh of wheels to the system
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

        sysFSI.AddFsiBody(wheel_body);
        if (i == 0 || i == 2) {
            sysSPH.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QuatFromAngleZ(CH_PI)), true);
        } else {
            sysSPH.AddPointsBCE(wheel_body, BCE_wheel, ChFrame<>(VNULL, QUNIT), true);
        }
    }
}

//------------------------------------------------------------------
// Function to save the povray files of the MBD
//------------------------------------------------------------------
void SaveParaViewFiles(double time, ChFsiSystemSPH& sysFSI) {
    ChSystem& sysMBS = sysFSI.GetMultibodySystem();

    std::string rover_dir = out_dir + "/rover";
    std::string filename;
    static int frame_number = -1;
    frame_number++;

    // save the VIPER body to obj/vtk files
    for (int i = 0; i < 1; i++) {
        auto body = rover->GetChassis()->GetBody();
        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = (GetChronoDataFile("robot/viper/obj/viper_chassis.obj"));
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/body_" + std::to_string(frame_number) + ".obj";
            std::vector<ChTriangleMeshConnected> meshes = {*mmesh};
            ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/body_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->GetCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->GetCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->GetIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->GetIndicesVertexes())
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

        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();      // body->GetPos();
        ChQuaternion<> body_rot = body_ref_frame.GetRot();  // body->GetRot();
        if (i == 0 || i == 2) {
            body_rot.Cross(body_rot, QuatFromAngleZ(CH_PI));
        }

        auto mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string obj_path = GetChronoDataFile(wheel_obj);
        double scale_ratio = 1.0;
        mmesh->LoadWavefrontMesh(obj_path, false, true);
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<ChTriangleMeshConnected> meshes = {*mmesh};
            ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/wheel_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->GetCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->GetCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->GetIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->GetIndicesVertexes())
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
        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();      // body->GetPos();
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
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/steerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<ChTriangleMeshConnected> meshes = {*mmesh};
            ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/steerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->GetCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->GetCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->GetIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->GetIndicesVertexes())
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
        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();      // body->GetPos();
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
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/lowerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<ChTriangleMeshConnected> meshes = {*mmesh};
            ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/lowerRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->GetCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->GetCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->GetIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->GetIndicesVertexes())
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
        ChFrame<> body_ref_frame = body->GetFrameRefToAbs();
        ChVector3d body_pos = body_ref_frame.GetPos();      // body->GetPos();
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
        mmesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        double mmass;
        ChVector3d mcog;
        ChMatrix33<> minertia;
        mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        mmesh->Transform(body_pos, ChMatrix33<>(body_rot));  // rotate the mesh based on the orientation of body

        if (save_obj) {  // save to obj file
            filename = rover_dir + "/upperRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".obj";
            std::vector<ChTriangleMeshConnected> meshes = {*mmesh};
            ChTriangleMeshConnected::WriteWavefront(filename, meshes);
        } else {  // save to vtk file
            filename = rover_dir + "/upperRod_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
            std::ofstream file;
            file.open(filename);
            file << "# vtk DataFile Version 2.0" << std::endl;
            file << "VTK from simulation" << std::endl;
            file << "ASCII" << std::endl;
            file << "DATASET UNSTRUCTURED_GRID" << std::endl;
            auto nv = mmesh->GetCoordsVertices().size();
            file << "POINTS " << nv << " float" << std::endl;
            for (auto& v : mmesh->GetCoordsVertices())
                file << v.x() << " " << v.y() << " " << v.z() << std::endl;
            auto nf = mmesh->GetIndicesVertexes().size();
            file << "CELLS " << nf << " " << 4 * nf << std::endl;
            for (auto& f : mmesh->GetIndicesVertexes())
                file << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
            file << "CELL_TYPES " << nf << std::endl;
            for (size_t ii = 0; ii < nf; ii++)
                file << "5 " << std::endl;
            file.close();
        }
    }

    // save box obstacle to vtk files
    double lx = 0.1;
    double ly = 0.25;
    double lz = 0.05;
    ChVector3d Node1 = ChVector3d(-lx, -ly, -lz);
    ChVector3d Node2 = ChVector3d(lx, -ly, -lz);
    ChVector3d Node3 = ChVector3d(lx, -ly, lz);
    ChVector3d Node4 = ChVector3d(-lx, -ly, lz);
    ChVector3d Node5 = ChVector3d(-lx, ly, -lz);
    ChVector3d Node6 = ChVector3d(lx, ly, -lz);
    ChVector3d Node7 = ChVector3d(lx, ly, lz);
    ChVector3d Node8 = ChVector3d(-lx, ly, lz);

    for (int i = 0; i < 2; i++) {
        filename = rover_dir + "/obstacle_" + std::to_string(i + 1) + "_" + std::to_string(frame_number) + ".vtk";
        std::ofstream file;
        file.open(filename);
        file << "# vtk DataFile Version 2.0" << std::endl;
        file << "VTK from simulation" << std::endl;
        file << "ASCII" << std::endl;
        file << "DATASET POLYDATA" << std::endl;

        file << "POINTS " << 8 << " "
             << "float" << std::endl;
        auto Body = sysMBS.GetBodies()[i + 2 + 16];
        ChVector3d center = Body->GetPos();
        ChMatrix33<> Rotation = Body->GetRot();
        ChVector3d vertex1 = Rotation * Node1 + center;
        ChVector3d vertex2 = Rotation * Node2 + center;
        ChVector3d vertex3 = Rotation * Node3 + center;
        ChVector3d vertex4 = Rotation * Node4 + center;
        ChVector3d vertex5 = Rotation * Node5 + center;
        ChVector3d vertex6 = Rotation * Node6 + center;
        ChVector3d vertex7 = Rotation * Node7 + center;
        ChVector3d vertex8 = Rotation * Node8 + center;
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
    for (int i = 1; i < sysMBS.GetBodies().size(); i++) {
        auto body = sysMBS.GetBodies()[i];
        ChFrame<> ref_frame = body->GetFrameRefToAbs();
        ChVector3d pos = ref_frame.GetPos();
        ChQuaternion<> rot = ref_frame.GetRot();
        ChVector3d vel = body->GetPosDt();

        std::string delim = ",";
        filename = rover_dir + "/body_pos_rot_vel" + std::to_string(i) + ".csv";
        std::ofstream file;
        if (sysMBS.GetChTime() > 0)
            file.open(filename, std::fstream::app);
        else {
            file.open(filename);
            file << "Time" << delim << "x" << delim << "y" << delim << "z" << delim << "q0" << delim << "q1" << delim
                 << "q2" << delim << "q3" << delim << "Vx" << delim << "Vy" << delim << "Vz" << std::endl;
        }

        file << sysMBS.GetChTime() << delim << pos.x() << delim << pos.y() << delim << pos.z() << delim << rot.e0()
             << delim << rot.e1() << delim << rot.e2() << delim << rot.e3() << delim << vel.x() << delim << vel.y()
             << delim << vel.z() << std::endl;

        file.close();
    }

    std::cout << "-------------------------------------" << std::endl;
    std::cout << " Output frame:  " << frame_number << std::endl;
    std::cout << " Time:          " << time << std::endl;
    std::cout << "-------------------------------------" << std::endl;
}
