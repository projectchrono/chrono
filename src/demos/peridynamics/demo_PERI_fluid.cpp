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
// Demo code explaining how to create a deformable solid using peridynamics. 
// Peridynamics allow an efficient handling of fracturing and cracks, working as
// a meshless framework: differently from conventional FEA it does not need 
// that the solid is preprocessed as a tetrahedral mesh. In the peridynamics 
// context, the solid is made with a node cloud, where nodes are connected
// each other (if withing a small distance called 'horizon') by microscopic bounds. 
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/fea/ChLinkNodeFrame.h"

#include "chrono_peridynamics/ChMatterPeriSprings.h"
#include "chrono_peridynamics/ChMatterPeriBulkElastic.h"
#include "chrono_peridynamics/ChMatterPeriLiquid.h"
#include "chrono_peridynamics/ChPeridynamics.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono_irrlicht/ChIrrTools.h"

#include "chrono_postprocess/ChBlender.h"


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

int main(int argc, char* argv[]) {
    assert(false); // ***THE PERIDYNAMIC FLUID FEATURE IS NOT YET TESTED***
    return 0;

    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Set small collision envelopes for objects that will be created from now on
    ChCollisionModel::SetDefaultSuggestedEnvelope(0.0002);
    ChCollisionModel::SetDefaultSuggestedMargin(0.0002);
    mphysicalSystem.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);


    // CREATE A FLOOR
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(0.0f);

    auto mfloorBody = chrono_types::make_shared<ChBodyEasyBox>(20,1,20,1000,true,true,mat);
    mphysicalSystem.Add(mfloorBody);
    mfloorBody->SetFixed(true);
    mfloorBody->SetPos(ChVector3d(0, -7.5, 0));

    mfloorBody->GetVisualShape(0)->SetColor(ChColor(0.2f, 0.2f, 0.2f));


    // CREATE A SPHERE FALLING IN THE FLUID:
    /*
    auto msphere = chrono_types::make_shared<ChBodyEasySphere>(0.7, 23000, true,true, mat);
    mphysicalSystem.Add(msphere);
    msphere->SetPos(ChVector3d(0, -1.5, 0));
    msphere->SetPosDt(ChVector3d(0, -16.5, 0));
    */
    // CREATE THREE WALLS TO CONTAIN THE FLUID
    auto mwall = chrono_types::make_shared<ChBodyEasyBox>(3,2,0.2, 23000, true, true, mat);
    mphysicalSystem.Add(mwall);
    mwall->SetPos(ChVector3d(0, -6, -1.6));
    mwall = chrono_types::make_shared<ChBodyEasyBox>(3, 2, 0.2, 23000, true, true, mat);
    mphysicalSystem.Add(mwall);
    mwall->SetPos(ChVector3d(0, -6, 1.6));
    mwall = chrono_types::make_shared<ChBodyEasyBox>(0.2, 2, 3.4, 23000, true, true, mat);
    mphysicalSystem.Add(mwall);
    mwall->SetPos(ChVector3d(1.6, -6, 0));
    mwall = chrono_types::make_shared<ChBodyEasyBox>(0.2, 2, 2.4, 23000, true, true, mat);
    mphysicalSystem.Add(mwall);
    mwall->SetPos(ChVector3d(-1.6, -6, 0.5));


    // CREATE THE PERIDYNAMIC CONTINUUM


    // Create peridynamics material 
    // This is a viscous fluid with compressibility, working 
    // similarly to SPH.
    auto my_perimaterial = chrono_types::make_shared<ChMatterPeriLiquid>();
    my_perimaterial->viscosity = 0.01; 
    my_perimaterial->density = 1000;
    my_perimaterial->pressure_stiffness = 80;


    // IMPORTANT!
    // This contains all the peridynamics particles and their materials. 
    auto my_peridynamics = chrono_types::make_shared<ChPeridynamics>();
    mphysicalSystem.Add(my_peridynamics);

    my_peridynamics->AddMatter(my_perimaterial);

    // Use the FillBox easy way to create the set of nodes in the Peridynamics matter
    my_peridynamics->FillBox(
        my_perimaterial,
        ChVector3d(2.6, 1.8, 2.6),                        // size of box
        2.7 / 20.0,                                   // resolution step
        1000,                                         // initial density
        ChCoordsys<>(ChVector3d(0, -3.4, 0), QUNIT),  // position & rotation of box
        false,                                        // do a centered cubic lattice initial arrangement
        1.8,                                          // set the horizon radius (as multiples of step) 
        0.1);                                         // set the collision radius (as multiples of step) for interface particles

    // Fix to remove boundary state at sides, from FillBox - to be fixed in future
    for (const auto& node : my_peridynamics->GetNodes()) {
        node->is_boundary = false;
    }

    // Attach visualization to peridynamics. The realtime visualization will show 
    // nodes and bonds with dots, lines etc. Suggestion: use the Blender importer add-on 
    // for rendering properties in falsecolor and other advanced features.

    auto mglyphs_nodes = chrono_types::make_shared<ChVisualPeriLiquid>(my_perimaterial);
    my_peridynamics->AddVisualShape(mglyphs_nodes);
    mglyphs_nodes->SetGlyphsSize(0.06);
    mglyphs_nodes->AttachVelocity(0, 20, "Vel"); // postprocessing tools can exploit this. Also suggest a min-max for falsecolor rendering.



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
    vsys->AddCamera(ChVector3d(-6, 0.3, 2.3), ChVector3d(0, -4, 0));
    vsys->AddLight(ChVector3d(30, 30, 60), 120, ChColor(0.6f, 0.6f, 0.6f));
    vsys->AddLight(ChVector3d(40, 60, 30), 120, ChColor(0.6f, 0.6f, 0.6f));


    // Modify some setting of the physical system for the simulation, if you want
    if (mphysicalSystem.GetSolver()->IsIterative()) {
        mphysicalSystem.GetSolver()->AsIterative()->SetMaxIterations(25);
    }

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    // Set timestep, that is smaller and smaller as stiffness of material increases 
    // and or mesh spacing decreases.
    double timestep = 0.002;

    while (vsys->Run()) {
        vsys->BeginScene();

        vsys->Render();
        
        vsys->EndScene();
         
        mphysicalSystem.DoStepDynamics(timestep);
        
        if (mphysicalSystem.GetNumSteps() % 10 == 0)
            blender_exporter.ExportData();
    }


    return 0;
}
