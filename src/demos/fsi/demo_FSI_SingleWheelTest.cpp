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
// Author: Wei Hu, Radu Serban
// =============================================================================

#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/core/ChTimer.h"

#include "chrono_fsi/ChSystemFsi.h"

#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

// Chrono namespaces
using namespace chrono;
using namespace chrono::fsi;

// Physical properties of terrain particles
double iniSpacing = 0.01;
double kernelLength = 0.01;
double density = 1700.0;

// Dimension of the terrain container
double smalldis = 1.0e-9;
double bxDim = 5.0 + smalldis;
double byDim = 0.8 + smalldis;
double bzDim = 0.12 + smalldis;

// Size of the wheel
double wheel_radius = 0.47;
double wheel_slip = 0.0;
double wheel_AngVel = 1.0;
double total_mass = 105.22;
std::string wheel_obj = "vehicle/hmmwv/hmmwv_tire_coarse_closed.obj";

// Initial Position of wheel
ChVector3d wheel_IniPos(-bxDim / 2 + wheel_radius, 0.0, wheel_radius + bzDim + iniSpacing);
ChVector3d wheel_IniVel(0.0, 0.0, 0.0);

// Simulation time and stepsize
double total_time = 5.0;
double dT = 2.5e-4;

// linear actuator and angular actuator
auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();

// Save data as csv files to see the results off-line using Paraview
bool output = true;
int output_fps = 20;

// Output directories and settings
const std::string out_dir = GetChronoOutputPath() + "FSI_Single_Wheel_Test/";

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// Verbose terminal output
bool verbose_fsi = true;
bool verbose = true;

//------------------------------------------------------------------
// Function to save wheel to Paraview VTK files
//------------------------------------------------------------------
void WriteWheelVTK(const std::string& filename, ChTriangleMeshConnected& mesh, const ChFrame<>& frame) {
    std::ofstream outf;
    outf.open(filename);
    outf << "# vtk DataFile Version 2.0" << std::endl;
    outf << "VTK from simulation" << std::endl;
    outf << "ASCII" << std::endl;
    outf << "DATASET UNSTRUCTURED_GRID" << std::endl;
    outf << "POINTS " << mesh.GetCoordsVertices().size() << " "
         << "float" << std::endl;
    for (auto& v : mesh.GetCoordsVertices()) {
        auto w = frame.TransformPointLocalToParent(v);
        outf << w.x() << " " << w.y() << " " << w.z() << std::endl;
    }
    auto nf = mesh.GetIndicesVertexes().size();
    outf << "CELLS " << nf << " " << 4 * nf << std::endl;
    for (auto& f : mesh.GetIndicesVertexes()) {
        outf << "3 " << f.x() << " " << f.y() << " " << f.z() << std::endl;
    }
    outf << "CELL_TYPES " << nf << std::endl;
    for (int i = 0; i < nf; i++) {
        outf << "5 " << std::endl;
    }
    outf.close();
}

//------------------------------------------------------------------
// Create the objects of the MBD system. Rigid bodies, and if FSI,
// their BCE representation are created and added to the systems
//------------------------------------------------------------------
void CreateSolidPhase(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    // Common contact material
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(1e8);
    cmaterial->SetFriction(0.9f);
    cmaterial->SetRestitution(0.4f);
    cmaterial->SetAdhesion(0);

    // Create a container -- always FIRST body in the system
    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    chrono::utils::AddBoxContainer(ground, cmaterial,                              //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(0, 0, -1),                           //
                                   false);
    ground->EnableCollision(true);

    // Add BCE particles attached on the walls into FSI system
    sysFSI.AddBoxContainerBCE(ground,                                     //
                              ChFrame<>(ChVector3d(0, 0, bzDim), QUNIT),  //
                              ChVector3d(bxDim, byDim, 2 * bzDim),        //
                              ChVector3i(2, 0, -1));

    // Create the wheel -- always SECOND body in the system
    auto trimesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    double scale_ratio = 1.0;
    trimesh->LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    trimesh->Transform(ChVector3d(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
    trimesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

    // Compute mass inertia from mesh
    double mmass;
    double mdensity = density;
    ChVector3d mcog;
    ChMatrix33<> minertia;
    trimesh->ComputeMassProperties(true, mmass, mcog, minertia);
    ChMatrix33<> principal_inertia_rot;
    ChVector3d principal_I;
    ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);
    mcog = ChVector3d(0.0, 0.0, 0.0);

    // Set the abs orientation, position and velocity
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> wheel_Rot = QUNIT;

    // Set the COG coordinates to barycenter, without displacing the REF reference.
    // Make the COG frame a principal frame.
    wheel->SetFrameCOMToRef(ChFrame<>(mcog, principal_inertia_rot));

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(mdensity * principal_I);
    wheel->SetPosDt(wheel_IniVel);
    wheel->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // Set the absolute position of the body:
    wheel->SetFrameRefToAbs(ChFrame<>(ChVector3d(wheel_IniPos), ChQuaternion<>(wheel_Rot)));
    sysMBS.AddBody(wheel);

    wheel->SetFixed(false);
    auto wheel_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(cmaterial, trimesh, false, false, 0.005);
    wheel->AddCollisionShape(wheel_shape);
    wheel->EnableCollision(false);

    // Add this body to the FSI system
    std::vector<ChVector3d> BCE_wheel;
    sysFSI.CreateMeshPoints(*trimesh, iniSpacing, BCE_wheel);
    sysFSI.AddPointsBCE(wheel, BCE_wheel, ChFrame<>(), true);
    sysFSI.AddFsiBody(wheel);

    // Create the chassis -- always THIRD body in the system
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 2.0);
    chassis->SetPos(wheel->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);

    // Add geometry of the chassis.
    chrono::utils::AddBoxGeometry(chassis.get(), cmaterial, ChVector3d(0.2, 0.2, 0.2), ChVector3d(0, 0, 0));
    sysMBS.AddBody(chassis);

    // Create the axle -- always FOURTH body in the system
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);

    // Add geometry of the axle.
    chrono::utils::AddSphereGeometry(axle.get(), cmaterial, 0.5, ChVector3d(0, 0, 0));
    sysMBS.AddBody(axle);

    // Connect the chassis to the containing bin (ground) through a translational joint and create a linear actuator.
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    double velocity = wheel_AngVel * wheel_radius * (1.0 - wheel_slip);
    auto actuator_fun = chrono_types::make_shared<ChFunctionRamp>(0.0, velocity);

    actuator->Initialize(ground, chassis, false, ChFrame<>(chassis->GetPos(), QUNIT),
                         ChFrame<>(chassis->GetPos() + ChVector3d(1, 0, 0), QUNIT));
    actuator->SetName("actuator");
    actuator->SetDistanceOffset(1);
    actuator->SetActuatorFunction(actuator_fun);
    sysMBS.AddLink(actuator);

    // Connect the axle to the chassis through a vertical translational joint.
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a engine joint.
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);
}

// =============================================================================

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
    if (!filesystem::create_directory(filesystem::path(out_dir + "/vtk"))) {
        std::cerr << "Error creating directory " << out_dir + "/vtk" << std::endl;
        return 1;
    }

    // Create the MBS and FSI systems
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    ChVector3d gravity = ChVector3d(0, 0, -9.81);
    sysMBS.SetGravitationalAcceleration(gravity);
    sysFSI.SetGravitationalAcceleration(gravity);

    sysFSI.SetVerbose(verbose_fsi);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_SingleWheelTest.json");
    if (argc == 3) {
        inputJson = std::string(argv[1]);
        wheel_slip = std::stod(argv[2]);
    } else if (argc != 1) {
        std::cout << "usage: ./demo_FSI_SingleWheelTest <json_file> <wheel_slip>" << std::endl;
        std::cout << "or to use default input parameters ./demo_FSI_SingleWheelTest " << std::endl;
        return 1;
    }

    sysFSI.ReadParametersFromFile(inputJson);

    // Set the initial particle spacing
    sysFSI.SetInitialSpacing(iniSpacing);

    // Set the SPH kernel length
    sysFSI.SetKernelLength(kernelLength);

    // Set the terrain density
    sysFSI.SetDensity(density);

    // Set the simulation stepsize
    sysFSI.SetStepSize(dT);

    // Set the terrain container size
    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(1.0e2);

    // Setup the SPH method
    sysFSI.SetSPHMethod(SPHMethod::WCSPH);

    // Set up the periodic boundary condition (if not, set relative larger values)
    ChVector3d cMin(-bxDim / 2 * 10, -byDim / 2 - 0.5 * iniSpacing, -bzDim * 10);
    ChVector3d cMax(bxDim / 2 * 10, byDim / 2 + 0.5 * iniSpacing, bzDim * 10);
    sysFSI.SetBoundaries(cMin, cMax);

    // Initialize the SPH particles
    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d boxCenter(0.0, 0.0, bzDim / 2);
    ChVector3d boxHalfDim(bxDim / 2 - initSpace0, byDim / 2, bzDim / 2 - initSpace0);
    sysFSI.AddBoxSPH(boxCenter, boxHalfDim);

    // Create Solid region and attach BCE SPH particles
    CreateSolidPhase(sysMBS, sysFSI);

    // Set simulation data output length
    sysFSI.SetOutputLength(0);

    // Construction of the FSI system must be finalized before running
    sysFSI.Initialize();

    auto wheel = sysMBS.GetBodies()[1];

    // Save wheel mesh
    ChTriangleMeshConnected wheel_mesh;
    wheel_mesh.LoadWavefrontMesh(GetChronoDataFile(wheel_obj), false, true);
    wheel_mesh.RepairDuplicateVertexes(1e-9);

    // Write the information into a txt file
    std::ofstream myFile;
    std::ofstream myDBP_Torque;
    if (output) {
        myFile.open(out_dir + "/results.txt", std::ios::trunc);
        myDBP_Torque.open(out_dir + "/DBP_Torque.txt", std::ios::trunc);
    }

    // Create a run-tme visualizer
#ifdef CHRONO_OPENGL
    ChFsiVisualizationGL fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI single wheel demo");
        fsi_vis.AddCamera(ChVector3d(0, -5 * byDim, 5 * bzDim), ChVector3d(0, 0, 0));
        fsi_vis.SetCameraMoveScale(0.05f);
        fsi_vis.EnableBoundaryMarkers(true);
        fsi_vis.Initialize();
    }
#endif

    // Start the simulation
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < total_time) {
        // Get the infomation of the wheel
        const auto& reaction = actuator->GetReaction2();
        const auto& force = reaction.force;
        const auto& torque = reaction.torque;
        const auto& w_pos = wheel->GetPos();
        const auto& w_vel = wheel->GetPosDt();
        const auto& angvel = wheel->GetAngVelLocal();

        if (verbose) {
            std::cout << "time: " << time << std::endl;
            std::cout << "  wheel position:         " << w_pos << std::endl;
            std::cout << "  wheel linear velocity:  " << w_vel << std::endl;
            std::cout << "  wheel angular velocity: " << angvel << std::endl;
            std::cout << "  drawbar pull:           " << force << std::endl;
            std::cout << "  wheel torque:           " << torque << std::endl;
        }

        if (output) {
            myFile << time << "\t" << w_pos.x() << "\t" << w_pos.y() << "\t" << w_pos.z() << "\t" << w_vel.x() << "\t"
                   << w_vel.y() << "\t" << w_vel.z() << "\t" << angvel.x() << "\t" << angvel.y() << "\t" << angvel.z()
                   << "\t" << force.x() << "\t" << force.y() << "\t" << force.z() << "\t" << torque.x() << "\t"
                   << torque.y() << "\t" << torque.z() << "\n";
            myDBP_Torque << time << "\t" << force.x() << "\t" << torque.z() << "\n";
        }

        if (output && time >= out_frame / output_fps) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/wheel." + std::to_string(counter++) + ".vtk";
            WriteWheelVTK(filename, wheel_mesh, wheel->GetFrameRefToAbs());
            out_frame++;
        }

        // Render SPH particles
#ifdef CHRONO_OPENGL
        if (render && time >= render_frame / render_fps) {
            if (!fsi_vis.Render())
                break;
            render_frame++;
        }
#endif

        // Call the FSI solver
        sysFSI.DoStepDynamics_FSI();
        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    if (output) {
        myFile.close();
        myDBP_Torque.close();
    }

    return 0;
}
