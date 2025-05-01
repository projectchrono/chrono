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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code explaining how to create a deformable solid using peridynamics
// with an implicit material, so that larger time steps can be used.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/functions/ChFunctionConstAcc.h"
#include "chrono/functions/ChFunctionRotationAxis.h"
#include "chrono/functions/ChFunctionPositionXYZFunctions.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/physics/ChLinkMotionImposed.h"
#include "chrono/fea/ChLinkNodeFrame.h"
#include "chrono/solver/ChSolverBB.h"
#include "chrono/solver/ChSolverADMM.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChElementHexaCorot_8.h"

#include "chrono_peridynamics/ChMatterPeriSprings.h"
#include "chrono_peridynamics/ChMatterPeriBB.h"
#include "chrono_peridynamics/ChMatterPeriBBimplicit.h"
#include "chrono_peridynamics/ChMatterPeriLinearElastic.h"
//#include "chrono_peridynamics/ChMatterPeriLiquid.h"
#include "chrono_peridynamics/ChPeridynamics.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "chrono_postprocess/ChBlender.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"


// Use the namespaces of Chrono

using namespace chrono;
using namespace peridynamics;
using namespace postprocess;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace chrono::irrlicht;


// SOME GLOBAL SETTINGS FOR THE BENCHMARKS



#define CANT_FEA_X 12
#define CANT_FEA_Y 6
#define CANT_FEA_Z 6




int test_cantilever_push(int argc, char* argv[], bool do_fea, bool do_peri) {

    double size_x = 1.2;
    double size_y = 0.6;
    double size_z = 0.6;
    double CANT_PERI_X = 24;

    double x_disp = size_x * 0.05;
    std::cout << "Benchmark: displace on x by " << x_disp << " [m]\n";

    if (do_fea) {
        // Create a Chrono::Engine physical system
        ChSystemSMC sys;

        // Create a mesh, that is a container for groups of elements and their referenced nodes.
        auto mesh = chrono_types::make_shared<ChMesh>();

        // Create a material, that must be assigned to each element and set its parameters
        auto material = chrono_types::make_shared<ChContinuumElastic>();
        material->SetYoungModulus(5e6);  // rubber 10e6, steel 200e9
        material->SetPoissonRatio(0.25);
        material->SetRayleighDampingBeta(0.001);
        material->SetDensity(1000);

        // Add some HEXAHEDRONS (isoparametric bricks)
        double sx = size_x / CANT_FEA_X;
        double sy = size_y / CANT_FEA_Y;
        double sz = size_z / CANT_FEA_Z;

        ChVector3d hexpos(0, -size_y / 2, -size_z / 2);
        ChMatrix33<> hexrot(QuatFromAngleY(0));

        std::array < std::array < std::array < std::shared_ptr<ChNodeFEAxyz>, CANT_FEA_Z + 1>, CANT_FEA_Y + 1>, CANT_FEA_X + 1> node_grid;

        for (int ix = 0; ix < node_grid.size(); ++ix) {
            for (int iy = 0; iy < node_grid[0].size(); ++iy) {
                for (int iz = 0; iz < node_grid[0][0].size(); ++iz) {
                    auto hnode = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(ix * sx, iy * sy, iz * sz));
                    node_grid[ix][iy][iz] = hnode;
                    mesh->AddNode(hnode);
                }
            }
        }
        for (int ix = 0; ix < node_grid.size() - 1; ++ix) {
            for (int iy = 0; iy < node_grid[0].size() - 1; ++iy) {
                for (int iz = 0; iz < node_grid[0][0].size() - 1; ++iz) {
                    auto hnode1 = node_grid[ix][iy][iz];
                    auto hnode2 = node_grid[ix + 1][iy][iz];
                    auto hnode3 = node_grid[ix + 1][iy + 1][iz];
                    auto hnode4 = node_grid[ix][iy + 1][iz];
                    auto hnode5 = node_grid[ix][iy][iz + 1];
                    auto hnode6 = node_grid[ix + 1][iy][iz + 1];
                    auto hnode7 = node_grid[ix + 1][iy + 1][iz + 1];
                    auto hnode8 = node_grid[ix][iy + 1][iz + 1];
                    auto helement1 = chrono_types::make_shared<ChElementHexaCorot_8>();
                    helement1->SetNodes(hnode1, hnode2, hnode3, hnode4, hnode5, hnode6, hnode7, hnode8);
                    helement1->SetMaterial(material);
                    mesh->AddElement(helement1);
                }
            }
        }

        // Add the mesh to the system
        sys.Add(mesh);

        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create a weight
        auto hammer = chrono_types::make_shared<ChBodyEasyBox>(size_x * 0.2, size_y * 1.5, size_z * 1.5,
            3000,      // density
            true,      // visualization?
            false      // collision?
        );
        hammer->SetPos(ChVector3d(size_x + 0.1 * size_x, 0, 0));
        sys.Add(hammer);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < mesh->GetNumNodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(inode))) {
                if (node->GetPos().x() < 0.01) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > (size_x - 0.01)) {
                    auto constraint = chrono_types::make_shared<ChLinkNodeFrameGeneric>(true, false, false);
                    constraint->Initialize(node, hammer);
                    sys.Add(constraint);
                }
            }
        }
        // Motor constraint
        auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
        motor->Initialize(truss, hammer, ChFrame<>(hammer->GetPos(), QuatFromAngleY(CH_PI_2)));
        auto motion_law = chrono_types::make_shared<ChFunctionConstAcc>(x_disp, 0.1, 0.9, 2.5);
        motor->SetMotionFunction(motion_law);
        sys.Add(motor);

        // Visualization of the FEM mesh.
        {
            // Mesh visualization - speed
            auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
            vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
            vis_mesh->SetColorscaleMinMax(0.0, 5.50);
            vis_mesh->SetShrinkElements(true, 0.90);
            vis_mesh->SetSmoothFaces(true);
            mesh->AddVisualShapeFEA(vis_mesh);
        }

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht FEM visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);
        //mesh->SetAutomaticGravity(true);

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(1);
        mkl_solver->UseSparsityPatternLearner(false);
        mkl_solver->LockSparsityPattern(false);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);

        sys.Update();

        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_cantilever_push_fea.txt");

        // Simulation loop
        while (vis.Run() && sys.GetChTime() < 3) {
            vis.BeginScene();
            vis.Render();
            vis.EndScene();
            sys.DoStepDynamics(0.01);

            my_output << sys.GetChTime() << ", " << motor->GetMotorForce() << ", " << motor->GetMotorPos() << ", " << motor->GetMotorPosDt() << "\n";
        }
    }

    ////////////////////////////////////////////


    if (do_peri) {
        // Create a Chrono::Engine physical system
        ChSystemNSC sys;

        // Set small collision envelopes for objects that will be created from now on
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
        ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        ChVector3d hexpos(0, 0, 0);
        ChMatrix33<> hexrot(QuatFromAngleY(0));

        auto my_perimaterial = chrono_types::make_shared<ChMatterPeriBB>();
        my_perimaterial->k_bulk = 5e6 * (2. / 3.);  // bulk stiffness (unit N/m^2)  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterial->damping = 0;               // rayleigh beta damping 
        //    my_perimaterial->max_stretch_fracture = 0.5; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //    my_perimaterial->max_stretch_break = 0.6; //  beyond this, bull break happens (bonds removed, collision surface generated)

            // IMPORTANT!
            // This contains all the peridynamics particles and their materials. 
        auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
        sys.Add(my_peridynamics);

        my_peridynamics->AddMatter(my_perimaterial);

        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        my_peridynamics->FillBox(
            my_perimaterial,
            ChVector3d(size_x, size_y, size_z),           // size of box
            size_x / CANT_PERI_X,                         // resolution step
            1000,                                         // initial density
            ChCoordsys<>(ChVector3d(size_x / 2, 0, 0), QUNIT),  // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.2);                                         // set the collision radius (as multiples of step) for interface particles


        // Attach visualization to peridynamics. 

        auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriBB>(my_perimaterial);
        my_peridynamics->AddVisualShape(mglyphs_nodes);
        mglyphs_nodes->SetGlyphsSize(0.01);
        mglyphs_nodes->AttachVelocity(0, 20, "Vel");


        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create a weight
        auto hammer = chrono_types::make_shared<ChBodyEasyBox>(size_x * 0.2, size_y * 1.5, size_z * 1.5,
            3000,      // density
            true,      // visualization?
            false      // collision?
        );
        hammer->SetPos(ChVector3d(size_x + 0.1 * size_x, 0, 0));
        sys.Add(hammer);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < my_peridynamics->GetNnodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_peridynamics->GetNodes()[inode])) {
                if (node->GetPos().x() < size_x / CANT_PERI_X - 0.01) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > (size_x - size_x / CANT_PERI_X - 0.01)) {
                    auto constraint = chrono_types::make_shared<ChLinkNodeFrameGeneric>(true, false, false);
                    constraint->Initialize(node, hammer);
                    sys.Add(constraint);
                }
            }
        }
        // Motor constraint

        auto motor = chrono_types::make_shared<ChLinkMotorLinearPosition>();
        motor->Initialize(truss, hammer, ChFrame<>(hammer->GetPos(), QuatFromAngleY(CH_PI_2)));
        auto motion_law = chrono_types::make_shared<ChFunctionConstAcc>(x_disp, 0.1, 0.9, 2.5);
        motor->SetMotionFunction(motion_law);
        sys.Add(motor);


        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht Peridynamics visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);

        // Modify some setting of the physical system for the simulation, if you want
        sys.SetSolverType(ChSolver::Type::PSOR);
        if (sys.GetSolver()->IsIterative()) {
            sys.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(false);
            sys.GetSolver()->AsIterative()->EnableWarmStart(true);
            sys.GetSolver()->AsIterative()->SetMaxIterations(10);
        }


        // IMPORTANT call this to generate bonds between nodes!
        ChPeridynamics::SetupInitialBonds(&sys, my_peridynamics);

        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_cantilever_push_peri.txt");

        // Simulation loop
        while (vis.Run() && sys.GetChTime() < 3) {
            vis.BeginScene();
            vis.Render();
            vis.EndScene();
            sys.DoStepDynamics(0.0005);

            my_output << sys.GetChTime() << ", " << motor->GetMotorForce() << ", " << motor->GetMotorPos() << ", " << motor->GetMotorPosDt() << "\n";
        }

    }

    return 0;
}





/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


int test_cantilever_torsion(int argc, char* argv[], bool do_fea, bool do_peri) {

    double size_x = 1.2;
    double size_y = 0.6;
    double size_z = 0.6;
    double CANT_PERI_X = 24;

    if (do_fea) {
        // Create a Chrono::Engine physical system
        ChSystemSMC sys;

        // Create a mesh, that is a container for groups of elements and their referenced nodes.
        auto mesh = chrono_types::make_shared<ChMesh>();

        // Create a material, that must be assigned to each element and set its parameters
        auto material = chrono_types::make_shared<ChContinuumElastic>();
        material->SetYoungModulus(5e6);//5e6);  // rubber 10e6, steel 200e9
        material->SetPoissonRatio(0.25);
        material->SetRayleighDampingBeta(0.001);
        material->SetDensity(1000);

        // Add some HEXAHEDRONS (isoparametric bricks)
        double sx = size_x / CANT_FEA_X;
        double sy = size_y / CANT_FEA_Y;
        double sz = size_z / CANT_FEA_Z;

        ChVector3d hexpos(0, -size_y / 2, -size_z / 2);
        ChMatrix33<> hexrot(QuatFromAngleY(0));

        std::array < std::array < std::array < std::shared_ptr<ChNodeFEAxyz>, CANT_FEA_Z + 1>, CANT_FEA_Y + 1>, CANT_FEA_X + 1> node_grid;

        for (int ix = 0; ix < node_grid.size(); ++ix) {
            for (int iy = 0; iy < node_grid[0].size(); ++iy) {
                for (int iz = 0; iz < node_grid[0][0].size(); ++iz) {
                    auto hnode = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(ix * sx, iy * sy, iz * sz));
                    node_grid[ix][iy][iz] = hnode;
                    mesh->AddNode(hnode);
                }
            }
        }
        for (int ix = 0; ix < node_grid.size() - 1; ++ix) {
            for (int iy = 0; iy < node_grid[0].size() - 1; ++iy) {
                for (int iz = 0; iz < node_grid[0][0].size() - 1; ++iz) {
                    auto hnode1 = node_grid[ix][iy][iz];
                    auto hnode2 = node_grid[ix + 1][iy][iz];
                    auto hnode3 = node_grid[ix + 1][iy + 1][iz];
                    auto hnode4 = node_grid[ix][iy + 1][iz];
                    auto hnode5 = node_grid[ix][iy][iz + 1];
                    auto hnode6 = node_grid[ix + 1][iy][iz + 1];
                    auto hnode7 = node_grid[ix + 1][iy + 1][iz + 1];
                    auto hnode8 = node_grid[ix][iy + 1][iz + 1];
                    auto helement1 = chrono_types::make_shared<ChElementHexaCorot_8>();
                    helement1->SetNodes(hnode1, hnode2, hnode3, hnode4, hnode5, hnode6, hnode7, hnode8);
                    helement1->SetMaterial(material);
                    mesh->AddElement(helement1);
                }
            }
        }

        // Add the mesh to the system
        sys.Add(mesh);

        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create a weight
        auto hammer = chrono_types::make_shared<ChBodyEasyBox>(size_x * 0.2, size_y * 1.5, size_z * 1.5,
            3000,      // density
            true,      // visualization?
            false      // collision?
        );
        hammer->SetPos(ChVector3d(size_x + 0.1 * size_x, 0, 0));
        sys.Add(hammer);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < mesh->GetNumNodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(inode))) {
                if (node->GetPos().x() < 0.01) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > (size_x - 0.01)) {
                    auto constraint = chrono_types::make_shared<ChLinkNodeFrame>();
                    constraint->Initialize(node, hammer);
                    sys.Add(constraint);
                }
            }
        }
        // Motor constraint
        auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        motor->Initialize(truss, hammer, ChFrame<>(hammer->GetPos(), QuatFromAngleY(CH_PI_2)));
        auto motion_law = chrono_types::make_shared<ChFunctionConstAcc>(15 * CH_DEG_TO_RAD, 0.1, 0.9, 2.5);
        motor->SetAngleFunction(motion_law);
        sys.Add(motor);

        // Visualization of the FEM mesh.
        {
            // Mesh visualization - speed
            auto vis_mesh = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
            vis_mesh->SetFEMdataType(ChVisualShapeFEA::DataType::NODE_SPEED_NORM);
            vis_mesh->SetColorscaleMinMax(0.0, 5.50);
            vis_mesh->SetShrinkElements(true, 0.90);
            vis_mesh->SetSmoothFaces(true);
            mesh->AddVisualShapeFEA(vis_mesh);
        }

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht FEM visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);
        //mesh->SetAutomaticGravity(true);

        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(1);
        mkl_solver->UseSparsityPatternLearner(false);
        mkl_solver->LockSparsityPattern(false);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);

        sys.Update();

        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_torsion_fea.txt");

        // Simulation loop
        while (vis.Run() && sys.GetChTime() < 3) {
            vis.BeginScene();
            vis.Render();
            vis.EndScene();
            sys.DoStepDynamics(0.005);

            my_output << sys.GetChTime() << ", " << motor->GetMotorTorque() << ", " << motor->GetMotorAngleDt() << ", " << motor->GetMotorAngle() << "\n";
        }

    }

    ////////////////////

    if (do_peri) {
        
        // Create a Chrono::Engine physical system
        ChSystemSMC sys;

        // Set small collision envelopes for objects that will be created from now on
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
        ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        ChVector3d hexpos(0, 0, 0);
        ChMatrix33<> hexrot(QuatFromAngleY(0));

        auto my_perimaterial = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterial->k_bulk = 5e6 * (2. / 3.);  // bulk stiffness (unit N/m^2)  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterial->damping = 0;               // Rayleigh beta damping 
        //    my_perimaterial->max_stretch_fracture = 0.5; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //    my_perimaterial->max_stretch_break = 0.6; //  beyond this, bull break happens (bonds removed, collision surface generated)

            // IMPORTANT!
            // This contains all the peridynamics particles and their materials. 
        auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
        sys.Add(my_peridynamics);

        my_peridynamics->AddMatter(my_perimaterial);

        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        my_peridynamics->FillBox(
            my_perimaterial,
            ChVector3d(size_x, size_y, size_z),           // size of box
            size_x / CANT_PERI_X,                         // resolution step
            1000,                                         // initial density
            ChCoordsys<>(ChVector3d(size_x / 2, 0, 0), QUNIT),  // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.2);                                         // set the collision radius (as multiples of step) for interface particles


        // Attach visualization to peridynamics. 

        auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterial);
        my_peridynamics->AddVisualShape(mglyphs_nodes);
        mglyphs_nodes->SetGlyphsSize(0.01);
        mglyphs_nodes->AttachVelocity(0, 20, "Vel");


        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create a weight
        auto hammer = chrono_types::make_shared<ChBodyEasyBox>(size_x * 0.2, size_y * 1.5, size_z * 1.5,
            3000,      // density
            true,      // visualization?
            false      // collision?
        );
        hammer->SetPos(ChVector3d(size_x + 0.1 * size_x, 0, 0));
        sys.Add(hammer);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < my_peridynamics->GetNnodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_peridynamics->GetNodes()[inode])) {
                if (node->GetPos().x() < size_x / CANT_PERI_X - 0.01) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > (size_x - size_x / CANT_PERI_X - 0.01)) {
                    auto constraint = chrono_types::make_shared<ChLinkNodeFrame>();
                    constraint->Initialize(node, hammer);
                    sys.Add(constraint);
                }
            }
        }

        // Motor constraint
        auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        motor->Initialize(truss, hammer, ChFrame<>(hammer->GetPos(), QuatFromAngleY(CH_PI_2)));
        auto motion_law = chrono_types::make_shared<ChFunctionConstAcc>(15 * CH_DEG_TO_RAD, 0.1, 0.9, 2.5);
        motor->SetAngleFunction(motion_law);
        sys.Add(motor);

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht Peridynamics visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);

        // Modify some setting of the physical system for the simulation, if you want
        sys.SetSolverType(ChSolver::Type::PSOR);
        if (sys.GetSolver()->IsIterative()) {
            sys.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(true);
            sys.GetSolver()->AsIterative()->EnableWarmStart(true);
            sys.GetSolver()->AsIterative()->SetMaxIterations(550);
            sys.GetSolver()->AsIterative()->SetTolerance(1e-5);
        }
        
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(12);
        mkl_solver->UseSparsityPatternLearner(false);
        mkl_solver->LockSparsityPattern(true);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);
        
        
        // IMPORTANT call this to generate bonds between nodes!
        ChPeridynamics::SetupInitialBonds(&sys, my_peridynamics);


        // -----Blender postprocess, optional
        // Create an exporter to Blender
        ChBlender blender_exporter = ChBlender(&sys);
        blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI_TORSION");
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0), 50);  // pos, aim, angle
        blender_exporter.ExportScript();



        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_torsion_peri.txt");

        ChTimer cputime;

        // Simulation loop
        while (vis.Run() && sys.GetChTime() < 3) {
            vis.BeginScene();
            vis.Render();
            vis.EndScene();
            cputime.start(); 
            sys.DoStepDynamics(0.04);
            cputime.stop();

            my_output << sys.GetChTime() << ", " << motor->GetMotorTorque() << ", " << motor->GetMotorAngleDt() << ", " << motor->GetMotorAngle() << "\n";

            if (sys.GetNumSteps() % 1 == 0)
                blender_exporter.ExportData();
        }

        std::cout << "\n\n  Tot time DoStepDynamics T=" << cputime.GetTimeSeconds()
            << "[s],  n steps=" << sys.GetNumSteps()
            << "  t/n_step=" << cputime.GetTimeMilliseconds() / sys.GetNumSteps() << "[ms] \n\n";

    }

    

    return 0;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////


int test_cantilever_crack(int argc, char* argv[], bool do_collisiononly, bool do_peri) {

    double size_x = 1.2;
    double size_y = 0.6;
    double size_z = 0.6;
    double CANT_PERI_X = 50;

    size_x = 1.2;
    size_y = 0.6;
    size_z = 0.3;

    if (do_collisiononly) {
    }

    ////////////////////

    if (do_peri) {

        // Create a Chrono::Engine physical system
        ChSystemNSC sys;

        // Set small collision envelopes for objects that will be created from now on
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
        ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        ChVector3d hexpos(0, 0, 0);
        ChMatrix33<> hexrot(QuatFromAngleY(0));

        auto my_perimaterialA = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterialA->k_bulk = 6e6 * (2. / 3.);  // bulk stiffness (unit N/m^2)  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialA->damping = 0.0002;               // Rayleigh beta damping 
        //    my_perimaterial->max_stretch_fracture = 0.02; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //    my_perimaterial->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)

        auto my_perimaterialB = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterialB->k_bulk = 6e6 * (2. / 3.);  // bulk stiffness (unit N/m^2)  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialB->damping = 0.0002;               // Rayleigh beta damping 
        //my_perimaterialB->max_stretch = 0.02; //  beyond this, fracture happens (bonds still in place, but become unilateral)
    //    my_perimaterialB->max_stretch_fracture = 0.016; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialB->max_stretch_break = 0.9; //  beyond this, bull break happens (bonds removed, collision surface generated)


            // IMPORTANT!
            // This contains all the peridynamics particles and their materials. 
        auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
        sys.Add(my_peridynamics);

        my_peridynamics->AddMatter(my_perimaterialA);
        my_peridynamics->AddMatter(my_perimaterialB);

        std::vector<std::pair<std::shared_ptr<ChMatterPeriBase>, double>> layers{ 
            {my_perimaterialA, size_y *0.5},
            {my_perimaterialB, size_y *0.5 + 0.01} 
        };

        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        my_peridynamics->FillBox(
            layers,
            ChVector3d(size_y, size_x, size_z),           // size of box
            size_x / CANT_PERI_X,                         // resolution step
            1000,                                         // initial density
            ChCoordsys<>(ChVector3d(size_x / 2, 0, 0), Q_ROTATE_X_TO_Y), // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.4);                                         // set the collision radius (as multiples of step) for interface particles


        // Attach visualization to peridynamics. 
        
        auto mglyphs_nodesA = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterialA);
        mglyphs_nodesA->SetColor(ChColor(0, 1, 0.5));
        mglyphs_nodesA->SetGlyphsSize(0.02);
        mglyphs_nodesA->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesA);
        
        auto mglyphs_nodesB = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterialB);
        mglyphs_nodesB->SetColor(ChColor(0, 0, 1));
        mglyphs_nodesB->SetGlyphsSize(0.022);
        mglyphs_nodesB->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesB);
        
        auto mglyphs_bondsA = chrono_types::make_shared<ChVisualPeriBBimplicitBonds>(my_perimaterialA);
        mglyphs_bondsA->draw_active = true;
        mglyphs_bondsA->draw_broken = false;
        mglyphs_bondsA->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsA);
        
        auto mglyphs_bondsB = chrono_types::make_shared<ChVisualPeriBBimplicitBonds>(my_perimaterialB);
        mglyphs_bondsB->draw_active = true;
        mglyphs_bondsB->draw_broken = false;
        mglyphs_bondsB->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsB);
        
        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create a weight
        auto hammer = chrono_types::make_shared<ChBodyEasyBox>(size_x * 0.2, size_y * 1.5, size_z * 1.5,
            3000,      // density
            true,      // visualization?
            false      // collision?
        );
        hammer->SetPos(ChVector3d(size_x + 0.1 * size_x, 0, 0));
        sys.Add(hammer);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < my_peridynamics->GetNnodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_peridynamics->GetNodes()[inode])) {
                if (node->GetPos().x() < size_x / CANT_PERI_X - 0.01) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > (size_x - size_x / CANT_PERI_X - 0.01)) {
                    auto constraint = chrono_types::make_shared<ChLinkNodeFrame>();
                    constraint->Initialize(node, hammer);
                    sys.Add(constraint);
                }
            }
        }

        // Motor constraint
        auto motor = chrono_types::make_shared<ChLinkMotionImposed>();
        motor->Initialize(truss, hammer, ChFrame<>(hammer->GetPos(), QuatFromAngleY(0)));
        auto motion_rot1 = chrono_types::make_shared<ChFunctionConstAcc>(14 * CH_DEG_TO_RAD, 0.15, 0.85, 2.5);
        auto motion_rot2 = chrono_types::make_shared<ChFunctionConstAcc>(-14*2 * CH_DEG_TO_RAD, 0.15, 0.85, 3.0);
        auto motion_cons = chrono_types::make_shared<ChFunctionConst>(0);
        auto motion_rot_seq = chrono_types::make_shared<ChFunctionSequence>();
        motion_rot_seq->InsertFunct(motion_rot1, 2.5);
        motion_rot_seq->InsertFunct(motion_rot2, 3.0, 1, true);
        motion_rot_seq->InsertFunct(motion_cons, 5.0, 1, true);
        auto mrot = chrono_types::make_shared<ChFunctionRotationAxis>();
        mrot->SetFunctionAngle(motion_rot_seq);
        mrot->SetAxis(VECT_Z);
        auto motion_pos1 = chrono_types::make_shared<ChFunctionConstAcc>(0.16, 0.15, 0.85, 2.5);
        auto motion_pos2 = chrono_types::make_shared<ChFunctionConstAcc>(-0.16 * 2 , 0.15, 0.85, 3.0);
        auto motion_pos_seq = chrono_types::make_shared<ChFunctionSequence>();
        motion_pos_seq->InsertFunct(motion_pos1, 2.5);
        motion_pos_seq->InsertFunct(motion_pos2, 3.0, 1, true);
        motion_pos_seq->InsertFunct(motion_cons, 5.0, 1, true);
        auto mpos = chrono_types::make_shared<ChFunctionPositionXYZFunctions>();
        mpos->SetFunctionY(motion_pos_seq);
        motor->SetRotationFunction(mrot);
        motor->SetPositionFunction(mpos);
        sys.Add(motor);

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht Peridynamics visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);

        // Modify some setting of the physical system for the simulation, if you want
        sys.SetSolverType(ChSolver::Type::PSOR);
        if (sys.GetSolver()->IsIterative()) {
            sys.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(true);
            sys.GetSolver()->AsIterative()->EnableWarmStart(true);
            sys.GetSolver()->AsIterative()->SetMaxIterations(505);
            sys.GetSolver()->AsIterative()->SetTolerance(1e-5);
        }

        /*
        auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(24);
        mkl_solver->UseSparsityPatternLearner(false);
        mkl_solver->LockSparsityPattern(true);
        mkl_solver->SetVerbose(false);
        sys.SetSolver(mkl_solver);
        */

        sys.Update();


        // -----Blender postprocess, optional
        // Create an exporter to Blender
        ChBlender blender_exporter = ChBlender(&sys);
        blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI_BENDING");
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0), 50);  // pos, aim, angle
        blender_exporter.ExportScript();



        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_bending_peri.txt");

        ChTimer cputime;

        // IMPORTANT call this to generate bonds between nodes!
        ChPeridynamics::SetupInitialBonds(&sys, my_peridynamics);

        double cut = size_x * 0.5;
        for (auto& ibo : my_perimaterialB->GetMapOfBonds()) {
            if ((ibo.second.nodeA->GetPos().x() < cut && ibo.second.nodeB->GetPos().x() > cut) ||
                (ibo.second.nodeB->GetPos().x() < cut && ibo.second.nodeA->GetPos().x() > cut)) {
                    ibo.second.force_density_val = 0;
                    ibo.second.state = ChMatterDataPerBondBBimplicit::bond_state::FRACTURED;
                    ibo.second.constraint.SetBoxedMinMax(0, 1e30); // enable complementarity, reaction>0.
                    
                    if (false) { // CASE with simple collisions
                        ibo.second.nodeA->is_boundary = true;
                        ibo.second.nodeB->is_boundary = true;
                        ibo.second.nodeA->coll_rad = size_x / CANT_PERI_X;
                        ibo.second.nodeB->coll_rad = size_x / CANT_PERI_X;
                        ibo.second.state = ChMatterDataPerBondBBimplicit::bond_state::BROKEN;
                    }
            }
        }
        my_perimaterialB->ComputeCollisionStateChanges();

        // Simulation loop
        while (vis.Run() && sys.GetChTime() < 5.8) {
            vis.BeginScene();
            vis.Render();
            vis.EndScene();
            cputime.start();
            sys.DoStepDynamics(0.015);//0.0004);
            cputime.stop();

            my_output << sys.GetChTime() << ", " << motor->GetReaction1().force.y() << ", "  << motor->GetReaction1().torque.z() 
                << ", " << motion_rot_seq->GetVal(sys.GetChTime()) << ", " << motion_rot_seq->GetDer(sys.GetChTime()) 
                << ", " << motion_pos_seq->GetVal(sys.GetChTime()) << ", " << motion_pos_seq->GetDer(sys.GetChTime())
                << "\n";

            if (sys.GetNumSteps() % 2 == 0)
                blender_exporter.ExportData();
        }

        std::cout << "\n\n  Tot time DoStepDynamics T=" << cputime.GetTimeSeconds()
            << "[s],  n steps=" << sys.GetNumSteps()
            << "  t/n_step=" << cputime.GetTimeMilliseconds() / sys.GetNumSteps() << "[ms] \n\n";

    }



    return 0;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////





int test_cantilever_fracture(int argc, char* argv[], bool do_peri) {

    double CANT_PERI_X = 50;

    // assuming units: [mm],[g],[ms]
    // L = [mm] = 1e-3[m]    M = [g] = 1e-3[kg]   T = [ms] = 1e-3 [s] 
    double size_x = 16; //[mm]
    double size_y = 3;  //[mm]
    double size_z = 10;  //[mm]
    double sphere_radius = 3; //[mm]
    double sphere_start_y = size_y * 0.5+ sphere_radius*1.05;
    ChVector3d sphere_velocity(0, -150, 0);  // [mm]/[ms] = [m]/[s] = [km/h]/3.6

    if (do_peri) {

        // Create a Chrono::Engine physical system
        ChSystemNSC sys;

        // Set small collision envelopes for objects that will be created from now on
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
        ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);


        auto my_perimaterialA = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterialA->k_bulk = 50e3 * (2. / 3.);   // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialA->damping = 0.0002;               // Rayleigh beta damping 
        //my_perimaterialA->max_stretch = 0.01; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialA->max_stretch_fracture = 0.01; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialA->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)

        auto my_perimaterialB = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterialB->k_bulk = 4e1 * (2. / 3.);   // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialB->damping = 0.0002;           // Rayleigh beta damping 
        //my_perimaterialB->max_stretch = 0.6;          //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialB->max_stretch_fracture = 0.6; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialB->max_stretch_break = 0.9; //  beyond this, bull break happens (bonds removed, collision surface generated)

        auto my_perimaterialC = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterialC->k_bulk = 16e3 * (2. / 3.);  // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialC->damping = 0.0002;               // Rayleigh beta damping 
        //my_perimaterialC->max_stretch = 0.05; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialC->max_stretch_fracture = 0.03; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        my_perimaterialC->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)


        // IMPORTANT!
        // This contains all the peridynamics particles and their materials. 
        auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
        sys.Add(my_peridynamics);

        my_peridynamics->AddMatter(my_perimaterialA);
        my_peridynamics->AddMatter(my_perimaterialB);
        my_peridynamics->AddMatter(my_perimaterialC);

        std::vector<std::pair<std::shared_ptr<ChMatterPeriBase>, double>> layers{
            {my_perimaterialA, size_y * 10.5},
            //{my_perimaterialB, size_y * 0.5 + 0.01}
        };

        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        my_peridynamics->FillBox(
            layers,
            ChVector3d(size_y, size_x, size_z),           // size of box
            size_x / CANT_PERI_X,                         // resolution step
            2200*1e-6,                                    // initial density [g/mm^3]=1e-6*[kg/m^3]
            ChCoordsys<>(ChVector3d(0, 0, 0), Q_ROTATE_X_TO_Y), // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.4);                                         // set the collision radius (as multiples of step) for interface particles


        my_peridynamics->FillSphere(
            my_perimaterialC,
            sphere_radius,                                // radius of sphere
            size_x / CANT_PERI_X,                         // resolution step
            11000 * 1e-6,                                 // initial density kg/mm^3
            ChCoordsys<>(ChVector3d(0, sphere_start_y, 0), Q_ROTATE_X_TO_Y), // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.4);


        // Attach visualization to peridynamics. 

        auto mglyphs_nodesA = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterialA);
        mglyphs_nodesA->SetColor(ChColor(0, 1, 0.5));
        mglyphs_nodesA->SetGlyphsSize(0.1);
        //mglyphs_nodesA->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesA);

        auto mglyphs_nodesB = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterialB);
        mglyphs_nodesB->SetColor(ChColor(0, 0, 1));
        mglyphs_nodesB->SetGlyphsSize(0.1);
        //mglyphs_nodesB->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesB);

        auto mglyphs_nodesC = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterialC);
        mglyphs_nodesC->SetColor(ChColor(0.5, 0.6, 0.6));
        mglyphs_nodesC->SetGlyphsSize(0.1);
        //mglyphs_nodesC->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesC);


        auto mglyphs_bondsA = chrono_types::make_shared<ChVisualPeriBBimplicitBonds>(my_perimaterialA);
        mglyphs_bondsA->draw_active = false;
        mglyphs_bondsA->draw_broken = false;
        mglyphs_bondsA->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsA);

        auto mglyphs_bondsB = chrono_types::make_shared<ChVisualPeriBBimplicitBonds>(my_perimaterialB);
        mglyphs_bondsB->draw_active = false;
        mglyphs_bondsB->draw_broken = false;
        mglyphs_bondsB->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsB);

        auto mglyphs_bondsC = chrono_types::make_shared<ChVisualPeriBBimplicitBonds>(my_perimaterialC);
        mglyphs_bondsC->draw_active = false;
        mglyphs_bondsC->draw_broken = false;
        mglyphs_bondsC->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsC);
        
        /*
        auto mglyphs_bondsA = chrono_types::make_shared<ChVisualPeriBBBonds>(my_perimaterialA);
        //mglyphs_bondsA->draw_active = false;
        //mglyphs_bondsA->draw_unbroken = true;
        mglyphs_bondsA->draw_broken = true;
        //mglyphs_bondsA->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsA);
        */

        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < my_peridynamics->GetNnodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_peridynamics->GetNodes()[inode])) {
                if (node->GetPos().x() < -0.5*size_x + size_x / CANT_PERI_X + 0.001) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > 0.5 * size_x - size_x / CANT_PERI_X - 0.001) {
                    node->SetFixed(true);
                }
                if (node->GetPos().y() > sphere_start_y-sphere_radius) {
                    node->SetPosDt(sphere_velocity);
                }
            }
        }

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht Peridynamics visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, size_y * 3, -15), ChVector3d(0, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);

        // Modify some setting of the physical system for the simulation, if you want
        sys.SetSolverType(ChSolver::Type::PSOR);
        if (sys.GetSolver()->IsIterative()) {
            sys.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(true);
            sys.GetSolver()->AsIterative()->EnableWarmStart(true);
            sys.GetSolver()->AsIterative()->SetMaxIterations(2);
            sys.GetSolver()->AsIterative()->SetTolerance(1e-5);
        }
        /*
        auto solver = chrono_types::make_shared<ChSolverADMM>(chrono_types::make_shared<ChSolverPardisoMKL>());
        solver->EnableWarmStart(true);
        solver->SetMaxIterations(2);
        solver->SetToleranceDual(1e-3);
        solver->SetTolerancePrimal(1e-3);
        solver->SetRho(1);
        solver->SetStepAdjustPolicy(ChSolverADMM::AdmmStepType::BALANCED_UNSCALED);
        sys.SetSolver(solver);
        */
        sys.Update();


        // -----Blender postprocess, optional
        // Create an exporter to Blender
        ChBlender blender_exporter = ChBlender(&sys);
        blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI_FRACTURE_foo");
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0), 50);  // pos, aim, angle
        blender_exporter.ExportScript();


        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_fracture.txt");

        // IMPORTANT call this to generate bonds between nodes!
        ChPeridynamics::SetupInitialBonds(&sys, my_peridynamics);



        // Simulation loop
        double end_time = 2.0;    // [ms]
        double dt = 0.00001; // [ms]
        ChTimer cputime;

        while (vis.Run() && sys.GetChTime() < end_time) {
            if (sys.GetNumSteps() % 1 == 0) {
                vis.BeginScene();
                vis.Render();
                vis.EndScene();
            }
            cputime.start();
            sys.DoStepDynamics(dt); // [ms]
            cputime.stop();

            if (sys.GetNumSteps() % 20 == 0)
                blender_exporter.ExportData();

        }

        std::cout << "\n\n  Tot time DoStepDynamics T=" << cputime.GetTimeSeconds()
            << "[s],  n steps=" << sys.GetNumSteps()
            << "  t/n_step=" << cputime.GetTimeMilliseconds() / sys.GetNumSteps() << "[ms] \n\n";
        std::cout << "\n\n  Tot nodes =" << my_peridynamics->GetNnodes() << "   " 
            << "  tot bonds =" << my_peridynamics->GetNproximities() << "\n"
            << "  tot constr =" << sys.GetNumConstraints() << "\n"
            << "  tot contacts =" << sys.GetNumContacts() << "\n";
    }

    

    return 0;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////



int test_cantilever_fracture_explicit(int argc, char* argv[], bool do_peri) {

    double CANT_PERI_X = 20;

    // assuming units: [mm],[g],[ms]
    // L = [mm] = 1e-3[m]    M = [g] = 1e-3[kg]   T = [ms] = 1e-3 [s] 
    double size_x = 16; //[mm]
    double size_y = 3;  //[mm]
    double size_z = 10;  //[mm]
    double sphere_radius = 3; //[mm]
    double sphere_start_y = size_y * 0.5 + sphere_radius * 1.05;
    ChVector3d sphere_velocity(0, -150, 0);  // [mm]/[ms] = [m]/[s] = [km/h]/3.6

    if (do_peri) {

        // Create a Chrono::Engine physical system
        ChSystemNSC sys;

        // Set small collision envelopes for objects that will be created from now on
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
        ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);


        auto my_perimaterialA = chrono_types::make_shared<ChMatterPeriBB>();
        my_perimaterialA->k_bulk = 50e3 * (2. / 3.);   // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialA->damping = 0.0002;               // Rayleigh beta damping 
        my_perimaterialA->max_stretch = 0.01; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialA->max_stretch_fracture = 0.01; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialA->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)

        auto my_perimaterialB = chrono_types::make_shared<ChMatterPeriBB>();
        my_perimaterialB->k_bulk = 4e1 * (2. / 3.);   // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialB->damping = 0.0002;           // Rayleigh beta damping 
        my_perimaterialB->max_stretch = 0.6;          //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialB->max_stretch_fracture = 0.6; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialB->max_stretch_break = 0.9; //  beyond this, bull break happens (bonds removed, collision surface generated)

        auto my_perimaterialC = chrono_types::make_shared<ChMatterPeriBB>();
        my_perimaterialC->k_bulk = 16e3 * (2. / 3.);  // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialC->damping = 0.0002;               // Rayleigh beta damping 
        my_perimaterialC->max_stretch = 0.05; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialC->max_stretch_fracture = 0.03; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialC->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)


        // IMPORTANT!
        // This contains all the peridynamics particles and their materials. 
        auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
        sys.Add(my_peridynamics);

        my_peridynamics->AddMatter(my_perimaterialA);
        my_peridynamics->AddMatter(my_perimaterialB);
        my_peridynamics->AddMatter(my_perimaterialC);

        std::vector<std::pair<std::shared_ptr<ChMatterPeriBase>, double>> layers{
            {my_perimaterialA, size_y * 10.5},
            //{my_perimaterialB, size_y * 0.5 + 0.01}
        };

        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        my_peridynamics->FillBox(
            layers,
            ChVector3d(size_y, size_x, size_z),           // size of box
            size_x / CANT_PERI_X,                         // resolution step
            2200 * 1e-6,                                    // initial density [g/mm^3]=1e-6*[kg/m^3]
            ChCoordsys<>(ChVector3d(0, 0, 0), Q_ROTATE_X_TO_Y), // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.4);                                         // set the collision radius (as multiples of step) for interface particles


        my_peridynamics->FillSphere(
            my_perimaterialC,
            sphere_radius,                                // radius of sphere
            size_x / CANT_PERI_X,                         // resolution step
            11000 * 1e-6,                                 // initial density kg/mm^3
            ChCoordsys<>(ChVector3d(0, sphere_start_y, 0), Q_ROTATE_X_TO_Y), // position & rotation of box
            1.7,                                          // set the horizon radius (as multiples of step) 
            0.4);


        // Attach visualization to peridynamics. 

        auto mglyphs_nodesA = chrono_types::make_shared<ChVisualPeriBB>(my_perimaterialA);
        mglyphs_nodesA->SetColor(ChColor(0, 1, 0.5));
        mglyphs_nodesA->SetGlyphsSize(0.1);
        //mglyphs_nodesA->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesA);

        auto mglyphs_nodesB = chrono_types::make_shared<ChVisualPeriBB>(my_perimaterialB);
        mglyphs_nodesB->SetColor(ChColor(0, 0, 1));
        mglyphs_nodesB->SetGlyphsSize(0.1);
        //mglyphs_nodesB->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesB);

        auto mglyphs_nodesC = chrono_types::make_shared<ChVisualPeriBB>(my_perimaterialC);
        mglyphs_nodesC->SetColor(ChColor(0.5, 0.6, 0.6));
        mglyphs_nodesC->SetGlyphsSize(0.1);
        //mglyphs_nodesC->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesC);

        
        auto mglyphs_bondsA = chrono_types::make_shared<ChVisualPeriBBBonds>(my_perimaterialA);
        mglyphs_bondsA->draw_unbroken = true;
        mglyphs_bondsA->draw_broken = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsA);
        

        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < my_peridynamics->GetNnodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_peridynamics->GetNodes()[inode])) {
                if (node->GetPos().x() < -0.5 * size_x + size_x / CANT_PERI_X + 0.001) {
                    node->SetFixed(true);
                }
                if (node->GetPos().x() > 0.5 * size_x - size_x / CANT_PERI_X - 0.001) {
                    node->SetFixed(true);
                }
                if (node->GetPos().y() > sphere_start_y - sphere_radius) {
                    node->SetPosDt(sphere_velocity);
                }
            }
        }

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht Peridynamics visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(size_x, size_y * 3, -15), ChVector3d(0, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);

        // Modify some setting of the physical system for the simulation, if you want
        sys.SetSolverType(ChSolver::Type::PSOR);
        if (sys.GetSolver()->IsIterative()) {
            sys.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(true);
            sys.GetSolver()->AsIterative()->EnableWarmStart(true);
            sys.GetSolver()->AsIterative()->SetMaxIterations(2);
            sys.GetSolver()->AsIterative()->SetTolerance(1e-5);
        }

        sys.Update();


        // -----Blender postprocess, optional
        // Create an exporter to Blender
        ChBlender blender_exporter = ChBlender(&sys);
        blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI_FRACTURE_EXPL");
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(size_x, 0.6, -1.5), ChVector3d(size_x / 2, 0, 0), 50);  // pos, aim, angle
        blender_exporter.ExportScript();


        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_fracture_expl.txt");

        // IMPORTANT call this to generate bonds between nodes!
        ChPeridynamics::SetupInitialBonds(&sys, my_peridynamics);



        // Simulation loop
        double end_time = 2.0;    // [ms]
        double dt = 0.00001; // [ms]

        while (vis.Run() && sys.GetChTime() < end_time) {
            if (sys.GetNumSteps() % 1 == 0) {
                vis.BeginScene();
                vis.Render();
                vis.EndScene();
            }
            sys.DoStepDynamics(dt); // [ms]

            /*
            my_output << sys.GetChTime() << ", " << motor->GetReaction1().force.y() << ", " << motor->GetReaction1().torque.z()
                << ", " << motion_rot_seq->GetVal(sys.GetChTime()) << ", " << motion_rot_seq->GetDer(sys.GetChTime())
                << ", " << motion_pos_seq->GetVal(sys.GetChTime()) << ", " << motion_pos_seq->GetDer(sys.GetChTime())
                << "\n";
            */
            if (sys.GetNumSteps() % 20 == 0)
                blender_exporter.ExportData();
        }
    }



    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////


void  FillTube(
    std::shared_ptr<ChPeridynamics> peri,
    std::shared_ptr<ChMatterPeriBase> mmatter, ///< matter to be used for this volume. (Matter must be added to ChPeridynamics too, via AddMatter(). )
    const double Rout,              ///< outer radius of the tube wall to fill
    const double Rin,               ///< outer radius of the tube wall to fill 
    const double L,                 ///< X length of the tube 
    const int samples_L,            ///< samples along L
    const int samples_R,            ///< samples along tube radial thickness
    const int samples_C,            ///< samples along circumference
    const double initial_density,  ///< density of the material inside the tube wall, for initialization of node's masses
    const ChCoordsys<> boxcoords,  ///< position and rotation of the box
    const double horizon_sfactor,  ///< the radius of horizon of the particle is spacing along X multiplied this value: horizon_sfactor*(L/samples_L)
    const double collision_sfactor ///< the radius of collision shape (sphere) of the particle is spacing along X multiplied this value: collision_sfactor*(L/samples_L)
)
{
    int totsamples = samples_L * samples_R * samples_C;

    double spacing_L = (L / samples_L);
    double horizon = horizon_sfactor * spacing_L;
    double collrad = collision_sfactor * spacing_L;
    //double mtotvol = L * (Rout*Rout - Rin*Rin) * CH_PI;
    //double mtotmass = mtotvol * initial_density;
    //double nodemass = mtotmass / (double)totsamples;
    //double nodevol = mtotvol / (double)totsamples;
    double totvol_sum = 0;
    for (int ix = 0; ix < samples_L; ix++)
        for (int ic = 0; ic < samples_C; ic++)
            for (int ir = 0; ir < samples_R; ir++) {
                double spacing_r = (Rout - Rin) / samples_R;
                double r = Rin + ir * spacing_r + 0.5 * spacing_r;
                double alpha = ic * CH_2PI / samples_C;
                ChVector3d pos(ix * spacing_L + 0.5 * spacing_L - 0.5 * L,
                    r * sin(alpha),
                    r * cos(alpha));
                ChVector3d mpos = boxcoords.TransformPointLocalToParent(pos);
                auto mnode = chrono_types::make_shared<ChNodePeri>();
                mnode->SetX0(mpos);
                mnode->SetPos(mpos);
                mnode->volume = spacing_L * (pow(Rin + spacing_r * (ir+1), 2) - pow(Rin + spacing_r * ir, 2)) * CH_PI/ samples_C;
                mnode->SetMass(mnode->volume*initial_density);
                mnode->is_fluid = false;
                mnode->coll_rad = collrad;
                mnode->h_rad = horizon;
                mnode->vol_size = std::pow(mnode->volume, 1 / 3.); // approx avg of 3 sides
                peri->AddNode(mnode);
                mmatter->AddNode(mnode);
                if ((ix == 0) || (ix == samples_L - 1) || (ir == 0) || (ir == samples_R - 1)) {
                    mnode->is_boundary = true;
                }
                //std::cout << "ix=" << ix << "ic=" << ic << "ir=" << ir << " vol=" << mnode->volume << " h=" << mnode->h_rad << " pos= " << pos << "\n" ;
                totvol_sum += mnode->volume; // only for cross checking
            }
}


int test_peristaltic(int argc, char* argv[], bool do_peri) {
    
    // assuming units: [mm],[g],[ms]
    // L = [mm] = 1e-3[m]    M = [g] = 1e-3[kg]   T = [ms] = 1e-3 [s] 

    // inputs
    int samples_L = 120;
    int samples_R = 3;
    double scale = 1.;
    double L         = scale * samples_L; //[mm]
    double Rin       = 0.04 * L;  //[mm]
    // computed
    double Rout = Rin + scale * samples_R;  //[mm]
    int samples_C = std::floor((Rin * CH_2PI) / (L / samples_L));

    if (do_peri) {

        // Create a Chrono::Engine physical system
        ChSystemNSC sys;

        // Set small collision envelopes for objects that will be created from now on
        ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
        ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
        sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

        // material for contacts
        auto contact_material = chrono_types::make_shared<ChContactMaterialNSC>();
        contact_material->SetFriction(0.4f);


        auto my_perimaterialA = chrono_types::make_shared<ChMatterPeriBBimplicit>();
        my_perimaterialA->k_bulk = 0.05e3 * (2. / 3.);   // bulk stiffness (unit [g]/([mm][ms]) = 1e-6*[N]/[mm^2])  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialA->damping = 0.0002;               // Rayleigh beta damping 
        //my_perimaterialA->max_stretch = 0.01; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialA->max_stretch_fracture = 0.01; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //my_perimaterialA->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)
        my_perimaterialA->SetContactMaterial(contact_material);

        // IMPORTANT!
        // This contains all the peridynamics particles and their materials. 
        auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
        sys.Add(my_peridynamics);

        my_peridynamics->AddMatter(my_perimaterialA);


        // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
        FillTube(
            my_peridynamics,
            my_perimaterialA,
            Rout, Rin, L,
            samples_L, samples_R, samples_C,
            1020 * 1e-6,                                    // initial density [g/mm^3]=1e-6*[kg/m^3]
            ChCoordsys<>(ChVector3d(0, 0, 0), QUNIT), // position & rotation of box
            1.8,                                          // set the horizon radius (as multiples of L step) 
            0.4);                                         // set the collision radius (as multiples of L step) for interface particles



        // Attach visualization to peridynamics. 

        auto mglyphs_nodesA = chrono_types::make_shared<ChVisualPeriBBimplicit>(my_perimaterialA);
        mglyphs_nodesA->SetColor(ChColor(0, 1, 0.5));
        mglyphs_nodesA->SetGlyphsSize(1);
        //mglyphs_nodesA->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesA);
        /*
        auto mglyphs_bondsA = chrono_types::make_shared<ChVisualPeriBBimplicitBonds>(my_perimaterialA);
        mglyphs_bondsA->draw_active = true;
        mglyphs_bondsA->draw_broken = false;
        mglyphs_bondsA->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsA);
        */

        // Create a truss
        auto truss = chrono_types::make_shared<ChBody>();
        truss->SetFixed(true);
        sys.Add(truss);

        // Create constraints between nodes and truss
        // (for example, fix to ground all nodes which are near y=0)
        for (unsigned int inode = 0; inode < my_peridynamics->GetNnodes(); ++inode) {
            if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_peridynamics->GetNodes()[inode])) {
                if (node->GetPos().x() < -0.5 * L + L / samples_L + 0.001) {
                    node->SetFixed(true);
                }
            }
        }

        // Create wheel and rollers
        

        double wheel_rad = 8*Rout;
        auto wheel = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, wheel_rad, Rout*2, 7000, true, false);
        wheel->SetPos(ChVector3d(0, wheel_rad -Rout, -Rout * 2-Rout));
        sys.Add(wheel);
        // Motor constraint
        auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
        motor->Initialize(truss, wheel, ChFrame<>(wheel->GetPos(), QuatFromAngleY(0)));
        auto motion_law = chrono_types::make_shared<ChFunctionConstAcc>(-120 * CH_DEG_TO_RAD, 0.05, 0.95, 3);
        motor->SetAngleFunction(motion_law);
        sys.Add(motor);
        int nrollers = 4;
        double phase = CH_PI/4;
        double roller_rad = Rout * 2;
        double roller_thick = Rout * 3;
        double roller_circ = wheel_rad - roller_rad - Rout*0.95;
        for (int i = 0; i < nrollers; ++i) {
            double alpha = phase + (double)i * CH_2PI / (double)nrollers;
            auto roller = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, roller_rad, roller_thick, 7000, true, true, contact_material);
            roller->SetPos(wheel->GetPos() + ChVector3d(0 + roller_circ*cos(alpha), roller_circ * sin(alpha), roller_thick));
            sys.Add(roller);
            auto revolute = chrono_types::make_shared<ChLinkMateGeneric>(true,  true, true, true, true, false);
            revolute->Initialize(roller, wheel, ChFrame<>(roller->GetPos(), QuatFromAngleY(0)));
            sys.Add(revolute);
        }
        auto base = chrono_types::make_shared<ChBodyEasyBox>(L, Rout, Rout * 4, 7000, true, true, contact_material);
        base->SetPos(ChVector3d(0, -Rout-Rout/2, 0));
        base->SetFixed(true);
        sys.Add(base);

        // Create the visualization system
        ChVisualSystemIrrlicht vis;
        vis.SetWindowSize(800, 600);
        vis.SetWindowTitle("Irrlicht Peridynamics visualization");
        vis.Initialize();
        vis.AddLogo();
        vis.AddSkyBox();
        vis.AddTypicalLights();
        vis.AddCamera(ChVector3d(L, 0.7 * L, 0.7*L), ChVector3d(0, 0, 0));
        vis.AttachSystem(&sys);

        sys.SetGravitationalAcceleration(VNULL);

        // Modify some setting of the physical system for the simulation, if you want
        sys.SetSolverType(ChSolver::Type::PSOR);
        if (sys.GetSolver()->IsIterative()) {
            sys.GetSolver()->AsIterative()->EnableDiagonalPreconditioner(true);
            sys.GetSolver()->AsIterative()->EnableWarmStart(true);
            sys.GetSolver()->AsIterative()->SetMaxIterations(20);
            sys.GetSolver()->AsIterative()->SetTolerance(1e-5);
        }


        sys.Update();


        // -----Blender postprocess, optional
        // Create an exporter to Blender
        ChBlender blender_exporter = ChBlender(&sys);
        blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI_PERISTALTIC");
        blender_exporter.AddAll();
        blender_exporter.SetCamera(ChVector3d(L, L, L), ChVector3d(0, 0, 0), 50);  // pos, aim, angle
        blender_exporter.ExportScript();


        std::ofstream my_output(GetChronoOutputPath() + "peridynamic_fracture.txt");

        // IMPORTANT call this to generate bonds between nodes!
        ChPeridynamics::SetupInitialBonds(&sys, my_peridynamics);



        // Simulation loop
        double end_time = 3.0;    // [ms]
        double dt = 0.001; // [ms]
        ChTimer cputime;

        while (vis.Run() && sys.GetChTime() < end_time) {
            if (sys.GetNumSteps() % 1 == 0) {
                vis.BeginScene();
                vis.Render();
                vis.EndScene();
            }
            cputime.start();
            sys.DoStepDynamics(dt); // [ms]
            cputime.stop();

            if (sys.GetNumSteps() % 30 == 0)
                blender_exporter.ExportData();

        }

        std::cout << "\n\n  Tot time DoStepDynamics T=" << cputime.GetTimeSeconds()
            << "[s],  n steps=" << sys.GetNumSteps()
            << "  t/n_step=" << cputime.GetTimeMilliseconds() / sys.GetNumSteps() << "[ms] \n\n";
        std::cout << "\n\n  Tot nodes =" << my_peridynamics->GetNnodes() << "   "
            << "  tot bonds =" << my_peridynamics->GetNproximities() << "\n"
            << "  tot constr =" << sys.GetNumConstraints() << "\n"
            << "  tot contacts =" << sys.GetNumContacts() << "\n";
    }



    return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {

    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;


    test_cantilever_push(argc, argv, false, false);

    test_cantilever_torsion(argc, argv, false, false);

    test_cantilever_crack(argc, argv, false, false);

    test_cantilever_fracture(argc, argv, false);

    test_cantilever_fracture_explicit(argc, argv, false);

    test_peristaltic(argc, argv, true);

    //test_1(argc, argv);
    
}