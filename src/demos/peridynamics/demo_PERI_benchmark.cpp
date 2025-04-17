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
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono/fea/ChElementHexaCorot_8.h"

#include "chrono_peridynamics/ChMatterPeriSprings.h"
#include "chrono_peridynamics/ChMatterPeriBulkElastic.h"
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


double size_x = 1.2;
double size_y = 0.6;
double size_z = 0.6;

#define CANT_FEA_X 12
#define CANT_FEA_Y 6
#define CANT_FEA_Z 6
#define CANT_PERI_X 24



int test_cantilever_push(int argc, char* argv[], bool do_fea, bool do_peri) {

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

        auto my_perimaterial = chrono_types::make_shared<ChMatterPeriBulkElastic>();
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

        auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriBulkElastic>(my_perimaterial);
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

        auto my_perimaterial = chrono_types::make_shared<ChMatterPeriBulkImplicit>();
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

        auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriBulkImplicit>(my_perimaterial);
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

#define CANT_PERI_X 30

int test_cantilever_fracture(int argc, char* argv[], bool do_collisiononly, bool do_peri) {

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

        auto my_perimaterialA = chrono_types::make_shared<ChMatterPeriBulkImplicit>();
        my_perimaterialA->k_bulk = 6e6 * (2. / 3.);  // bulk stiffness (unit N/m^2)  K=E/(3(1-2mu)) , with mu=1/4 -> K=E*(2/3)
        my_perimaterialA->damping = 0.0002;               // Rayleigh beta damping 
        //    my_perimaterial->max_stretch_fracture = 0.02; //  beyond this, fracture happens (bonds still in place, but become unilateral)
        //    my_perimaterial->max_stretch_break = 0.2; //  beyond this, bull break happens (bonds removed, collision surface generated)

        auto my_perimaterialB = chrono_types::make_shared<ChMatterPeriBulkImplicit>();
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
        
        auto mglyphs_nodesA = chrono_types::make_shared<ChVisualPeriBulkImplicit>(my_perimaterialA);
        mglyphs_nodesA->SetColor(ChColor(0, 1, 0.5));
        mglyphs_nodesA->SetGlyphsSize(0.02);
        mglyphs_nodesA->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesA);
        
        auto mglyphs_nodesB = chrono_types::make_shared<ChVisualPeriBulkImplicit>(my_perimaterialB);
        mglyphs_nodesB->SetColor(ChColor(0, 0, 1));
        mglyphs_nodesB->SetGlyphsSize(0.022);
        mglyphs_nodesB->draw_noncolliding = false;
        my_peridynamics->AddVisualShape(mglyphs_nodesB);
        
        auto mglyphs_bondsA = chrono_types::make_shared<ChVisualPeriBulkImplicitBonds>(my_perimaterialA);
        mglyphs_bondsA->draw_active = true;
        mglyphs_bondsA->draw_broken = false;
        mglyphs_bondsA->draw_fractured = true;
        my_peridynamics->AddVisualShape(mglyphs_bondsA);
        
        auto mglyphs_bondsB = chrono_types::make_shared<ChVisualPeriBulkImplicitBonds>(my_perimaterialB);
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
                    ibo.second.state = ChMatterDataPerBondBulkImplicit::bond_state::FRACTURED;
                    ibo.second.constraint.SetBoxedMinMax(0, 1e30); // enable complementarity, reaction>0.
                    
                    if (false) { // CASE with simple collisions
                        ibo.second.nodeA->is_boundary = true;
                        ibo.second.nodeB->is_boundary = true;
                        ibo.second.nodeA->coll_rad = size_x / CANT_PERI_X;
                        ibo.second.nodeB->coll_rad = size_x / CANT_PERI_X;
                        ibo.second.state = ChMatterDataPerBondBulkImplicit::bond_state::BROKEN;
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



/////////////////////////////////////////////////////////////////////////////////////////////////////




int main(int argc, char* argv[]) {

    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;


    test_cantilever_push(argc, argv, false, false);

    test_cantilever_torsion(argc, argv, false, false);

    test_cantilever_fracture(argc, argv, false, true);


    //test_1(argc, argv);
    
}