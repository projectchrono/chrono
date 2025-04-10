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
#include "chrono/physics/ChLinkMotorRotationAngle.h"
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

#define CANT_FEA_X 12
#define CANT_FEA_Y 7
#define CANT_FEA_Z 7

int test_cantilever_fea(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

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
    ChVector3d hexpos(0, 0, 0);
    double sx = 0.1;
    double sy = 0.1;
    double sz = 0.1;
    double size_x = sx * (CANT_FEA_X -1);
    double size_y = sy * (CANT_FEA_Y -1);
    double size_z = sz * (CANT_FEA_Z -1);

    std::shared_ptr<ChNodeFEAxyz> hnode1_lower;
    std::shared_ptr<ChNodeFEAxyz> hnode2_lower;
    std::shared_ptr<ChNodeFEAxyz> hnode3_lower;
    std::shared_ptr<ChNodeFEAxyz> hnode4_lower;

    ChMatrix33<> hexrot(QuatFromAngleY(0));

    std::array < std::array < std::array < std::shared_ptr<ChNodeFEAxyz>, CANT_FEA_Z>, CANT_FEA_Y>, CANT_FEA_X> node_grid;

    for (int ix = 0; ix < node_grid.size(); ++ix) {
        for (int iy = 0; iy < node_grid[0].size(); ++iy) {
            for (int iz = 0; iz < node_grid[0][0].size(); ++iz) {
                auto hnode = chrono_types::make_shared<ChNodeFEAxyz>(hexpos + hexrot * ChVector3d(ix * sx, iy * sy, iz * sz));
                node_grid[ix][iy][iz] = hnode;
                mesh->AddNode(hnode);
            }
        }
    }
    for (int ix = 0; ix < node_grid.size()-1; ++ix) {
        for (int iy = 0; iy < node_grid[0].size()-1; ++iy) {
            for (int iz = 0; iz < node_grid[0][0].size()-1; ++iz) {
                auto hnode1 = node_grid[ix][iy][iz];
                auto hnode2 = node_grid[ix+1][iy][iz];
                auto hnode3 = node_grid[ix+1][iy+1][iz];
                auto hnode4 = node_grid[ix][iy+1][iz];
                auto hnode5 = node_grid[ix][iy][iz+1];
                auto hnode6 = node_grid[ix+1][iy][iz+1];
                auto hnode7 = node_grid[ix+1][iy+1][iz+1];
                auto hnode8 = node_grid[ix][iy+1][iz+1];
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
    auto hammer = chrono_types::make_shared<ChBodyEasyBox>(size_x*0.2, size_y*1.5, size_z*1.5,
            3000,      // density
            true,      // visualization?
            false      // collision?
        );   
    hammer->SetPos(ChVector3d(size_x+0.1*size_x, size_y*0.5, size_z * 0.5));
    sys.Add(hammer);

    // Create constraints between nodes and truss
    // (for example, fix to ground all nodes which are near y=0)
    for (unsigned int inode = 0; inode < mesh->GetNumNodes(); ++inode) {
        if (auto node = std::dynamic_pointer_cast<ChNodeFEAxyz>(mesh->GetNode(inode))) {
            if (node->GetPos().x() < 0.01) {
                node->SetFixed(true);
            }
            if (node->GetPos().x() > (size_x-0.01)) {
                auto constraint = chrono_types::make_shared<ChLinkNodeFrame>();
                constraint->Initialize(node, hammer);
                sys.Add(constraint);
            }
        }
    }
    // Motor constraint
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->Initialize(truss, hammer, ChFrame<>(hammer->GetPos(),QuatFromAngleY(CH_PI_2)));
    auto motion_law = chrono_types::make_shared<ChFunctionConstAcc>(0.6, 0.1, 0.9, 2);
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
    {
        // Node visualization - positions
        auto vis_nodes = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
        vis_nodes->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
        vis_nodes->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
        vis_nodes->SetSymbolsThickness(0.006);
        mesh->AddVisualShapeFEA(vis_nodes);
    }

    // Create the visualization system
    ChVisualSystemIrrlicht vis;
    vis.SetWindowSize(800, 600);
    vis.SetWindowTitle("Irrlicht FEM visualization");
    vis.Initialize();
    vis.AddLogo();
    vis.AddSkyBox();
    vis.AddTypicalLights();
    vis.AddCamera(ChVector3d(0.0, 0.6, -1.0));
    vis.AttachSystem(&sys);

    sys.SetGravitationalAcceleration(VNULL);


    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>(1);
    mkl_solver->UseSparsityPatternLearner(false);
    mkl_solver->LockSparsityPattern(false);
    mkl_solver->SetVerbose(false);
    sys.SetSolver(mkl_solver);

    sys.Update();
    /*
    // Solver settings
    sys.SetTimestepperType(chrono::ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);

    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    sys.SetSolver(solver);
    solver->SetMaxIterations(40);
    solver->SetTolerance(1e-10);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetVerbose(false);
    */

    // Simulation loop
    while (vis.Run()) {
        vis.BeginScene();
        vis.Render();
        vis.EndScene();
        sys.DoStepDynamics(0.001);
    }

    return 0;
}



int test_1(int argc, char* argv[]) {

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Set small collision envelopes for objects that will be created from now on
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
    mphysicalSystem.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // CREATE A FLOOR
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.2f);

    auto mfloorBody = chrono_types::make_shared<ChBodyEasyBox>(20,1,20,1000,true,true,mat);
    mphysicalSystem.Add(mfloorBody);
    mfloorBody->SetFixed(true);
    mfloorBody->SetPos(ChVector3d(0, -7.5, 0));

    mfloorBody->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.2f));

    
    // CREATE A SPHERE PRESSING THE MATERIAL:
    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.7, 7000, true,true, mat);
    mphysicalSystem.Add(msphere);
    msphere->SetPos(ChVector3d(0, -1.5, 0));
    msphere->SetPosDt(ChVector3d(0, -20.5, 0));
    
   

    // CREATE THE PERIDYNAMIC CONTINUUM

    // Create peridynamics material 
    // This is a very simple one: a linear bond-based elastic material, defined
    // via the bulk elasticity modulus. The Poisson ratio is fixed to 1/4. 
    
    auto my_perimaterial = chrono_types::make_shared<ChMatterPeriBulkImplicit>();
    my_perimaterial->k_bulk = 300e6;       // bulk stiffness (unit N/m^2)
    my_perimaterial->r_bulk = 2000;        // damping 
    my_perimaterial->max_stretch_fracture = 0.1; // 0.001 beyond this, fracture happens (bonds still in place, but become unilateral)
    my_perimaterial->max_stretch_break    = 0.3; // 0.003 beyond this, bull break happens (bonds removed, collision surface generated)

    // IMPORTANT!
    // This contains all the peridynamics particles and their materials. 
    auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
    mphysicalSystem.Add(my_peridynamics);

    my_peridynamics->AddMatter(my_perimaterial);

    // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
    my_peridynamics->FillBox(
        my_perimaterial,
        ChVector3d(3, 1.5, 3),                        // size of box
        4.0 / 15.0,                                   // resolution step
        1000,                                         // initial density
        ChCoordsys<>(ChVector3d(0, -3.4, 0), QUNIT),  // position & rotation of box
        false,                                        // do a centered cubic lattice initial arrangement
        1.6,                                          // set the horizon radius (as multiples of step) 
        0.4);                                         // set the collision radius (as multiples of step) for interface particles

    // Just for testing, fix some nodes
    /*
    for (const auto& node : my_peridynamics->GetNodes()) {
        if (node->GetPos().z() < -2.70 || node->GetPos().z() > 2.50 || node->GetPos().x() < -2.70 || node->GetPos().x() > 2.50)
            node->SetFixed(true);
    }
    */

    // Attach visualization to peridynamics. The realtime visualization will show 
    // nodes and bonds with dots, lines etc. Suggestion: use the Blender importer add-on 
    // for rendering properties in falsecolor and other advanced features.
    
    auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriBulkImplicit>(my_perimaterial);
    my_peridynamics->AddVisualShape(mglyphs_nodes);
    mglyphs_nodes->SetGlyphsSize(0.04);
    mglyphs_nodes->AttachVelocity(0, 20, "Vel"); // postprocessing tools can exploit this. Also suggest a min-max for falsecolor rendering.
    
    auto mglyphs_bounds = chrono_types::make_shared<ChVisualPeriBulkImplicitBounds>(my_perimaterial);
    mglyphs_bounds->draw_active = false;
    mglyphs_bounds->draw_broken = true;
    mglyphs_bounds->draw_fractured = true;
    my_peridynamics->AddVisualShape(mglyphs_bounds);
    

    // -----Blender postprocess, optional

    // Create an exporter to Blender
    ChBlender blender_exporter = ChBlender(&mphysicalSystem);

    // Set the path where it will save all files, a directory will be created if not existing
    blender_exporter.SetBasePath(GetChronoOutputPath() + "BLENDER_PERI");

    // Export all existing visual shapes to POV-Ray
    blender_exporter.AddAll();

    blender_exporter.SetCamera(ChVector3d(3, 4, -5), ChVector3d(0, 0.5, 0), 50);  // pos, aim, angle

    blender_exporter.ExportScript();

    // --------------------
    

    // Create the Irrlicht visualization system
    auto vsys = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vsys->AttachSystem(&mphysicalSystem);
    vsys->SetWindowSize(1024, 768);
    vsys->SetWindowTitle("Peridynamics test");
    vsys->Initialize();
    vsys->AddLogo();
    vsys->AddSkyBox();
    vsys->AddCamera(ChVector3d(-3, -1, 1.3), ChVector3d(0, -4, 0));
    vsys->AddLight(ChVector3d(30, 30, 60), 120, ChColor(0.6f, 0.6f, 0.6f));
    vsys->AddLight(ChVector3d(40, 60, 30), 120, ChColor(0.6f, 0.6f, 0.6f));


    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    if (mphysicalSystem.GetSolver()->IsIterative()) {
        mphysicalSystem.GetSolver()->AsIterative()->SetMaxIterations(10);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // Set timestep, that is smaller and smaller as stiffness of material increases 
    // and or mesh spacing decreases.
    double timestep = 0.0002;

    while (vsys->Run()) {
        vsys->BeginScene();

        vsys->Render();
        
        vsys->EndScene();
        
        mphysicalSystem.DoStepDynamics(timestep);

        if (mphysicalSystem.GetNumSteps() % 2 == 0)
            blender_exporter.ExportData();
    }


    return 0;
}


int main(int argc, char* argv[]) {

    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    test_cantilever_fea(argc, argv);

    //test_1(argc, argv);

}