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
// FEA for 3D beams
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/collision/ChCCollisionSystemBullet.h"

#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_fea/ChLinkDirFrame.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;


// A helper function that creates a'lobed gear', almost a macro user in main() 
// to quickly create one or two rotating obstacles for the extruding beam 

std::shared_ptr<ChBody> CreateLobedGear (
            ChVector<> gear_center,
            int    lobe_copies,
            double lobe_width,
            double lobe_primitive_rad,
            double lobe_inner_rad,
            double lobe_outer_rad,
            double lobe_thickness,
            ChSystem& my_system,
            std::shared_ptr<ChMaterialSurface> mysurfmaterial
            )  {

    auto mgear = std::make_shared<ChBody>();
    mgear->SetMaterialSurface(mysurfmaterial);
    mgear->SetPos(gear_center);
    my_system.Add(mgear);

    // add cylindrical lobes
    mgear->GetCollisionModel()->ClearModel();
    for (int i = 0; i< lobe_copies; ++i) {
        double phase = CH_C_2PI * ((double)i/(double)lobe_copies);
        // this is a quick shortcut from ChUtilsCreators.h, 
        // it both adds the collision shape and the visualization asset:
        /*
        utils::AddCylinderGeometry(
            mgear.get(), 
            lobe_width*0.5, 
            lobe_thickness*0.5, 
            ChVector<>(lobe_primitive_rad*sin(phase), lobe_primitive_rad*cos(phase),0),
            Q_from_AngAxis(CH_C_PI_2, VECT_X), // rotate cylinder axis: from default on Y axis, to Z axis
            true);    
        */  
        utils::AddBoxGeometry(
            mgear.get(), 
            ChVector<>(lobe_width, lobe_outer_rad-lobe_inner_rad, lobe_thickness)*0.5, // half size used in this function 
            ChVector<>(0.5*(lobe_outer_rad+lobe_inner_rad)*sin(phase), 0.5*(lobe_outer_rad+lobe_inner_rad)*cos(phase),0),
            Q_from_AngAxis(-phase, VECT_Z), // rotate cylinder axis: from default on Y axis, to Z axis
            true);
    }
    utils::AddCylinderGeometry( mgear.get(), lobe_inner_rad, lobe_thickness*0.5,  ChVector<>(0,0,0), Q_from_AngAxis(CH_C_PI_2, VECT_X), true);
    mgear->GetCollisionModel()->BuildModel();
    mgear->SetCollide(true);
    
    return mgear;
}




int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Here set the inward-outward margins for collision shapes: should make sense in the scale of the model
    collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.002);
    collision::ChCollisionSystemBullet::SetContactBreakingThreshold(0.0001);


    // Create a ground object, useful reference for connecting constraints etc.
    auto mground = std::make_shared<ChBody>();
    mground->SetBodyFixed(true);
    my_system.Add(mground);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.

    auto my_mesh = std::make_shared<ChMesh>();
    my_system.Add(my_mesh);

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    auto msection_OLD = std::make_shared<ChBeamSectionAdvanced>();

    double wire_diameter = 0.012;
    msection_OLD->SetAsCircularSection(wire_diameter); 
	msection_OLD->SetYoungModulus(0.01e9);  // not exactly a steel wire...
	msection_OLD->SetGshearModulus(0.01e9 * 0.7);
	msection_OLD->SetBeamRaleyghDamping(0.1);

	auto melasticity = std::make_shared<ChElasticityCosseratSimple>();
	melasticity->SetYoungModulus(0.01e9);
	melasticity->SetGshearModulus(0.01e9 * 0.7);
	melasticity->SetBeamRaleyghDamping(0.1);
	auto msection = std::make_shared<ChBeamSectionCosserat>(melasticity);
	msection->SetDensity(1000);
	msection->SetAsCircularSection(wire_diameter);

    // Create the surface material for the contacts; this contains information about friction etc.
    // It is a SMC (penalty) material: interpenetration might happen for low Young stiffness,
    // but unstable simulation might happen for high stiffness, requiring smaller timesteps.
    
    /*
    // option A: Hertz contact force model
    my_system.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(12e3);  // to adjust heuristically..
    mysurfmaterial->SetRestitution(0.1f);
    mysurfmaterial->SetFriction(0.2f);
	*/

    
    // Option B: Hooke force model 
    my_system.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    my_system.UseMaterialProperties(false);
    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetKn(350); // contact normal stiffness
    mysurfmaterial->SetKt(350); // contact tangential stiffness
    mysurfmaterial->SetGn(20);   // contact normal damping
    mysurfmaterial->SetGt(20);   // contact tangential damping
    mysurfmaterial->SetFriction(0.2f);
    

    //
    // Add the EXTRUDER
    //

    auto extruder = std::make_shared<ChExtruderBeamEuler>(
            &my_system,                 // the physical system 
            my_mesh,                    // the mesh where to add the beams
            msection_OLD,                   // section for created beam
            0.020,                        // beam element length (size of discretization: the smaller, the more precise)
            ChCoordsys<>(ChVector<>(0,0,0)), // outlet coordinate system (x axis is the extrusion dir)
            0.04                         // the extrusion speed
            );

    // Enable collision for extruded beam
    extruder->SetContact( mysurfmaterial,  // the NSC material for contact surfaces
                          1.15*wire_diameter*0.5  // the radius of the collision spheres at the nodes, (enlarge 15%)
                          );
/*
    auto extruder = std::make_shared<ChExtruderBeamIGA>(
            &my_system,                 // the physical system 
            my_mesh,                    // the mesh where to add the beams
            msection,                   // section for created beam
            0.020,                        // beam element length (size of discretization: the smaller, the more precise)
            ChCoordsys<>(ChVector<>(0,0,0)), // outlet coordinate system (x axis is the extrusion dir)
            0.04,                        // the extrusion speed
            2                            // the order of beams
            );
*/
    // Enable collision for extruded beam
    extruder->SetContact( mysurfmaterial,  // the NSC material for contact surfaces
                          1.15*wire_diameter*0.5  // the radius of the collision spheres at the nodes, (enlarge 15%)
                          );

    //
    // Add some other beams 
    //
    // ***NOTE: hack! this is needed because if the extruder starts with 0 beams in the scene, the 
    //    ChVisualizationFEAmesh does not visualize any of the newly generated beams by extrusion. Must be fixed.

    double beam_L = 0.1;

    auto hnode1 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(-0.4, 0, 0)));
    auto hnode2 = std::make_shared<ChNodeFEAxyzrot>(ChFrame<>(ChVector<>(-0.4+beam_L, 0, 0)));
    my_mesh->AddNode(hnode1);
    my_mesh->AddNode(hnode2);

    auto belement1 = std::make_shared<ChElementBeamEuler>();
    belement1->SetNodes(hnode1, hnode2);
    belement1->SetSection(msection_OLD);

    my_mesh->AddElement(belement1);
    // Fix a node to ground - the easy way, without constraints
    hnode1->SetFixed(true);
    
	// We do not want gravity effect on FEA elements in this demo
	my_mesh->SetAutomaticGravity(true);

    //
    // Attach a visualization of the FEM mesh.
    //

    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.01);
    mvisualizebeamC->SetZbufferHide(false);
    my_mesh->AddAsset(mvisualizebeamC);


    //
    // Add some obstacles. two rotating lobed gears.
    //
    // Here create two rotating lobed gears, just for fun, that wil trap the 
    // extruded beam. To quickly create them, use the CreateLobedGear() function
    // implemented at the top of this file. 
    // Also, create two simple constant speed motors to rotate the lobed gears.

    int    lobe_copies = 8;
    double lobe_width = 0.03;
    double lobe_primitive_rad = 0.3;
    double lobe_inner_rad = 0.23;
    double lobe_outer_rad = 0.34;
    double lobe_thickness = 0.08;
    ChVector<> gear_centerLOW(0.4,-lobe_primitive_rad,0);
    ChVector<> gear_centerHI (0.4, lobe_primitive_rad,0);

    auto gearLOW =  CreateLobedGear (gear_centerLOW, lobe_copies, lobe_width, lobe_primitive_rad, 
                    lobe_inner_rad, lobe_outer_rad, lobe_thickness, my_system, mysurfmaterial); 

    auto mgear_motorLOW = std::make_shared<ChLinkMotorRotationSpeed>();
    mgear_motorLOW->Initialize(gearLOW, mground, ChFrame<>(gear_centerLOW));
    my_system.Add(mgear_motorLOW);

    auto mgear_speedLOW = std::make_shared<ChFunction_Const>(-0.1); // [rad/s]
    mgear_motorLOW->SetSpeedFunction(mgear_speedLOW);

    auto gearHI =  CreateLobedGear (gear_centerHI, lobe_copies, lobe_width, lobe_primitive_rad, 
                    lobe_inner_rad, lobe_outer_rad, lobe_thickness, my_system, mysurfmaterial);
    gearHI->SetRot(Q_from_AngZ(0.5*CH_C_2PI/lobe_copies)); // to phase half step respect to other gear 

    auto mgear_motorHI = std::make_shared<ChLinkMotorRotationSpeed>();
    mgear_motorHI->Initialize(gearHI, mground, ChFrame<>(gear_centerHI));
    my_system.Add(mgear_motorHI);

    auto mgear_speedHI = std::make_shared<ChFunction_Const>( 0.1); // [rad/s]
    mgear_motorHI->SetSpeedFunction(mgear_speedHI);



    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Beam continuous extrusion and FEA contacts", core::dimension2d<u32>(800, 600),
                         false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Mark completion of system construction
    my_system.SetupInitial();



    //
    // THE SOFT-REAL-TIME CYCLE
    //

    my_system.SetSolverType(ChSolver::Type::MINRES);
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(460);
    my_system.SetMaxItersSolverStab(460);
    my_system.SetTolForce(1e-13);
    auto msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver());
    msolver->SetVerbose(false);
    msolver->SetDiagonalPreconditioning(true);

    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
            
    application.SetTimestep(0.001);
    application.SetVideoframeSaveInterval(20);
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();
        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1,20,20,CSYSNORM, irr::video::SColor(255,100,100,100),true);

        application.DoStep();

        extruder->Update();    //***REMEMBER*** to do this to update the extrusion

        application.EndScene();
    }

    return 0;
}

