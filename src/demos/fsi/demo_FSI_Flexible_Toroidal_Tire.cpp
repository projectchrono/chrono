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
// Author: Pei Li, Wei Hu
// =============================================================================

#include <assert.h>
#include <stdlib.h>
#include <ctime>

#include "chrono/physics/ChSystemSMC.h"

#ifdef CHRONO_PARDISO_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"

#include "chrono/fea/ChElementShellANCF_3423.h"
#include "chrono/fea/ChLinkNodeSlopeFrame.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChMeshExporter.h"
#include "chrono/fea/ChBuilderBeam.h"

#include "chrono_fsi/ChSystemFsi.h"

#ifdef CHRONO_OPENGL
    #include "chrono_fsi/visualization/ChFsiVisualizationGL.h"
#endif

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::fsi;

// -----------------------------------------------------------------

// Set the output directory
std::string out_dir = GetChronoOutputPath() + "FSI_Flexible_Toroidal_Tire";

// Dimension of the domain
double smalldis = 1.0e-9;
double bxDim = 5.0 + smalldis;
double byDim = 0.6 + smalldis;
double bzDim = 0.3 + smalldis;

// Dimension of the fluid domain
double fxDim = 5.0 + smalldis;
double fyDim = 0.6 + smalldis;
double fzDim = 0.2 + smalldis;

// Size of the wheel
double wheel_radius = 0.35;
double wheel_slip = 0.0;
double wheel_AngVel = 1.0;
double total_mass = 105.22;

double m_rim_radius = 0.35;
double m_height = 0.195;
double m_thickness = 0.014;
unsigned int m_div_circumference = 30;
unsigned int m_div_width = 6;
double m_alpha = 0.15;

// Initial Position of wheel
ChVector3d wheel_IniPos(-bxDim / 2 + 1.5 * wheel_radius, 0.0, 1.5 * wheel_radius + bzDim);
ChVector3d wheel_IniVel(0.0, 0.0, 0.0);

// Simulation time
double t_end = 10.0;

// Output frequency
bool output = true;
double output_fps = 20;

// Enable/disable run-time visualization (if Chrono::OpenGL is available)
bool render = true;
float render_fps = 100;

// -----------------------------------------------------------------

std::shared_ptr<fea::ChMesh> Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI);

// -----------------------------------------------------------------

int main(int argc, char* argv[]) {
    // Create an MBS system and an FSI system
    ChSystemSMC sysMBS;
    ChSystemFsi sysFSI(&sysMBS);

    sysMBS.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Use the default input file or you may enter your input parameters as a command line argument
    std::string inputJson = GetChronoDataFile("fsi/input_json/demo_FSI_Flexible_Toroidal_Tire_Granular.json");
    if (argc == 1) {
        std::cout << "Use the default JSON file" << std::endl;
    } else if (argc == 2) {
        std::cout << "Use the specified JSON file" << std::endl;
        std::string my_inputJson = std::string(argv[1]);
        inputJson = my_inputJson;
    } else {
        std::cout << "usage: ./demo_FSI_Flexible_Toroidal_Tire_Granular <json_file>" << std::endl;
        return 1;
    }
    sysFSI.ReadParametersFromFile(inputJson);

    sysFSI.SetContainerDim(ChVector3d(bxDim, byDim, bzDim));

    auto initSpace0 = sysFSI.GetInitialSpacing();
    ChVector3d cMin = ChVector3d(-5 * bxDim, -byDim / 2.0 - initSpace0 / 2.0, -5 * bzDim);
    ChVector3d cMax = ChVector3d(5 * bxDim, byDim / 2.0 + initSpace0 / 2.0, 10 * bzDim);
    sysFSI.SetBoundaries(cMin, cMax);

    // Set SPH discretization type, consistent or inconsistent
    sysFSI.SetConsistentDerivativeDiscretization(false, false);

    // Set cohsion of the granular material
    sysFSI.SetCohesionForce(2000.0);

    // Create SPH particles of fluid region
    chrono::utils::ChGridSampler<> sampler(initSpace0);
    ChVector3d boxCenter(-bxDim / 2 + fxDim / 2, 0, fzDim / 2);
    ChVector3d boxHalfDim(fxDim / 2, fyDim / 2, fzDim / 2 - initSpace0);
    chrono::utils::ChGenerator::PointVector points = sampler.SampleBox(boxCenter, boxHalfDim);
    size_t numPart = points.size();
    for (int i = 0; i < numPart; i++) {
        sysFSI.AddSPHParticle(points[i]);
    }

    // Create solids
    auto mesh = Create_MB_FE(sysMBS, sysFSI);

    // Initialize FSI system
    sysFSI.Initialize();

    // Create oputput directories
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cerr << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    out_dir = out_dir + "/" + sysFSI.GetPhysicsProblemString() + "_" + sysFSI.GetSphMethodTypeString();
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

#ifdef CHRONO_OPENGL
    // Create a run-tme visualizer
    ChFsiVisualizationGL fsi_vis(&sysFSI);
    if (render) {
        fsi_vis.SetTitle("Chrono::FSI Flexible Toroidal Tire Demo");
        fsi_vis.AddCamera(ChVector3d(bxDim / 8, -3, 0.25), ChVector3d(bxDim / 8, 0.0, 0.25));
        fsi_vis.SetCameraMoveScale(1.0f);
        fsi_vis.EnableBoundaryMarkers(false);
        fsi_vis.Initialize();
    }
#endif

    // Set MBS solver
#ifdef CHRONO_PARDISO_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sysMBS.SetSolver(mkl_solver);
#else
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sysMBS.SetSolver(solver);
    solver->SetMaxIterations(2000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->SetVerbose(false);
#endif

    // Simulation loop
    double dT = sysFSI.GetStepSize();
    double time = 0.0;
    int sim_frame = 0;
    int out_frame = 0;
    int render_frame = 0;

    ChTimer timer;
    timer.start();
    while (time < t_end) {
        std::cout << sim_frame << " time: " << time << std::endl;

        if (output && time >= out_frame / output_fps) {
            std::cout << "-------- Output" << std::endl;
            sysFSI.PrintParticleToFile(out_dir + "/particles");
            sysFSI.PrintFsiInfoToFile(out_dir + "/fsi", time);
            static int counter = 0;
            std::string filename = out_dir + "/vtk/flex_body." + std::to_string(counter++) + ".vtk";
            fea::ChMeshExporter::WriteFrame(mesh, out_dir + "/Flex_MESH.vtk", filename);
            out_frame++;
        }

#ifdef CHRONO_OPENGL
        // Render SPH particles
        if (render && time >= render_frame / render_fps) {
            if (!fsi_vis.Render())
                break;
            render_frame++;
        }
#endif

        sysFSI.DoStepDynamics_FSI();

        time += dT;
        sim_frame++;
    }
    timer.stop();
    std::cout << "\nSimulation time: " << timer() << " seconds\n" << std::endl;

    return 0;
}

//--------------------------------------------------------------------
// Create the objects of the MBD system. Rigid/flexible bodies, and if
// fsi, their bce representation are created and added to the systems
std::shared_ptr<fea::ChMesh> Create_MB_FE(ChSystemSMC& sysMBS, ChSystemFsi& sysFSI) {
    sysMBS.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
    sysFSI.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    auto ground = chrono_types::make_shared<ChBody>();
    ground->SetFixed(true);
    sysMBS.AddBody(ground);

    // Bottom collision plate
    auto cmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    cmaterial->SetYoungModulus(6e4);
    cmaterial->SetFriction(0.3f);
    cmaterial->SetRestitution(0.2f);
    cmaterial->SetAdhesion(0);
    chrono::utils::AddBoxContainer(ground, cmaterial,                              //
                                   ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                                   ChVector3d(bxDim, byDim, bzDim), 0.1,           //
                                   ChVector3i(0, 0, -1),                           //
                                   false);
    ground->EnableCollision(true);

    // Fluid representation of walls
    sysFSI.AddBoxContainerBCE(ground,                                         //
                              ChFrame<>(ChVector3d(0, 0, bzDim / 2), QUNIT),  //
                              ChVector3d(bxDim, byDim, bzDim),                //
                              ChVector3i(2, 0, -1));

    // Create a wheel rigid body
    auto wheel = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> Body_rot = QUNIT;
    ChVector3d Body_pos = wheel_IniPos;
    ChVector3d Body_vel = wheel_IniVel;

    // Set inertia
    wheel->SetMass(total_mass * 1.0 / 2.0);
    wheel->SetInertiaXX(ChVector3d(60, 60, 60));
    wheel->SetPosDt(Body_vel);
    wheel->SetAngVelLocal(ChVector3d(0.0, 0.0, 0.0));  // set an initial anular velocity (rad/s)

    // Set the absolute position of the body:
    wheel->SetFrameRefToAbs(ChFrame<>(ChVector3d(Body_pos), ChQuaternion<>(Body_rot)));
    wheel->SetFixed(false);
    wheel->EnableCollision(false);
    sysMBS.AddBody(wheel);

    // Create the chassis
    auto chassis = chrono_types::make_shared<ChBody>();
    chassis->SetMass(total_mass * 1.0 / 2.0);
    chassis->SetPos(wheel->GetPos());
    chassis->EnableCollision(false);
    chassis->SetFixed(false);
    sysMBS.AddBody(chassis);

    // Create the axle
    auto axle = chrono_types::make_shared<ChBody>();
    axle->SetMass(total_mass * 1.0 / 2.0);
    axle->SetPos(wheel->GetPos());
    axle->EnableCollision(false);
    axle->SetFixed(false);
    sysMBS.AddBody(axle);

    // Connect the chassis to the ground through a translational joint and create a linear actuator
    auto prismatic1 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic1->Initialize(ground, chassis, ChFrame<>(chassis->GetPos(), QuatFromAngleY(CH_PI_2)));
    prismatic1->SetName("prismatic_chassis_ground");
    sysMBS.AddLink(prismatic1);

    ////double velocity = wheel_AngVel * wheel_radius * (1.0 - wheel_slip);
    ////auto actuator_fun = chrono_types::make_shared<ChFunctionRamp>(0.0, velocity);

    ////auto actuator = chrono_types::make_shared<ChLinkLockLinActuator>();
    ////actuator->Initialize(ground, chassis, false, ChFrame<>(chassis->GetPos(), QUNIT),
    ////                     ChFrame<>(chassis->GetPos() + ChVector3d(1, 0, 0), QUNIT));
    ////actuator->SetName("actuator");
    ////actuator->SetDistanceOffset(1);
    ////actuator->SetActuatorFunction(actuator_fun);
    ////sysMBS.AddLink(actuator);

    // Connect the axle to the chassis through a vertical translational joint
    auto prismatic2 = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic2->Initialize(chassis, axle, ChFrame<>(chassis->GetPos(), QUNIT));
    prismatic2->SetName("prismatic_axle_chassis");
    sysMBS.AddLink(prismatic2);

    // Connect the wheel to the axle through a motor link
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->SetName("engine_wheel_axle");
    motor->Initialize(wheel, axle, ChFrame<>(wheel->GetPos(), chrono::QuatFromAngleX(-CH_PI_2)));
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, wheel_AngVel));
    sysMBS.AddLink(motor);

    // Create an FEA mesh representing a cantilever plate modeled with ANCF shell elements
    auto mesh = chrono_types::make_shared<fea::ChMesh>();

    // Add the tire
    {
        auto mat = chrono_types::make_shared<ChMaterialShellANCF>(2000, 1.0e7, 0.3);

        // Create the mesh nodes
        // The nodes are first created in the wheel local frame, assuming Y as the tire axis,
        // and are then transformed to the global frame
        for (unsigned int i = 0; i < m_div_circumference; i++) {
            double phi = (CH_2PI * i) / m_div_circumference;
            for (unsigned int j = 0; j <= m_div_width; j++) {
                double theta = -CH_PI_2 + (CH_PI * j) / m_div_width;

                double x = (m_rim_radius + m_height * cos(theta)) * cos(phi) + wheel_IniPos.x();
                double y = m_height * sin(theta) + wheel_IniPos.y();
                double z = (m_rim_radius + m_height * cos(theta)) * sin(phi) + wheel_IniPos.z();
                ChVector3d loc = ChVector3d(x, y, z);

                double nx = cos(theta) * cos(phi);
                double ny = sin(theta);
                double nz = cos(theta) * sin(phi);
                ChVector3d dir = ChVector3d(nx, ny, nz);

                auto node = chrono_types::make_shared<ChNodeFEAxyzD>(loc, dir);
                node->SetMass(0.0);
                mesh->AddNode(node);

                // Fix the edge node on the wheel
                if (j == 0 || j == m_div_width) {
                    auto mlink = chrono_types::make_shared<ChLinkNodeFrame>();
                    mlink->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyzD>(node), wheel);
                    sysMBS.Add(mlink);
                }
            }
        }

        // Element thickness
        double dz = m_thickness;

        // Create the ANCF shell elements
        unsigned int num_elem = 0;
        for (unsigned int i = 0; i < m_div_circumference; i++) {
            for (unsigned int j = 0; j < m_div_width; j++) {
                // Adjacent nodes
                unsigned int inode0, inode1, inode2, inode3;
                inode1 = j + i * (m_div_width + 1);
                inode2 = j + 1 + i * (m_div_width + 1);
                if (i == m_div_circumference - 1) {
                    inode0 = j;
                    inode3 = j + 1;
                } else {
                    inode0 = j + (i + 1) * (m_div_width + 1);
                    inode3 = j + 1 + (i + 1) * (m_div_width + 1);
                }

                auto node0 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode0));
                auto node1 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode1));
                auto node2 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode2));
                auto node3 = std::dynamic_pointer_cast<ChNodeFEAxyzD>(mesh->GetNode(inode3));

                // Create the element and set its nodes
                auto element = chrono_types::make_shared<ChElementShellANCF_3423>();
                element->SetNodes(node0, node1, node2, node3);

                // Element dimensions
                double dx =
                    0.5 * ((node1->GetPos() - node0->GetPos()).Length() + (node3->GetPos() - node2->GetPos()).Length());
                double dy =
                    0.5 * ((node2->GetPos() - node1->GetPos()).Length() + (node3->GetPos() - node0->GetPos()).Length());

                // Set element dimensions
                element->SetDimensions(dx, dy);

                // Add a single layers with a fiber angle of 0 degrees
                element->AddLayer(dz, 0 * CH_DEG_TO_RAD, mat);

                // Set other element properties
                element->SetAlphaDamp(m_alpha);

                // Add element to mesh
                mesh->AddElement(element);

                ChVector3d center = 0.25 * (element->GetNodeA()->GetPos() + element->GetNodeB()->GetPos() +
                                            element->GetNodeC()->GetPos() + element->GetNodeD()->GetPos());
                std::cout << "Adding element" << num_elem << "  with center:  " << center.x() << " " << center.y()
                          << " " << center.z() << std::endl;

                num_elem++;
            }
        }
    }

    // Add the mesh to the MBS system
    sysMBS.Add(mesh);

    // Add the mesh to the FSI system (only these meshes interact with the fluid)
    sysFSI.AddFsiMesh(mesh);

    return mesh;
}
