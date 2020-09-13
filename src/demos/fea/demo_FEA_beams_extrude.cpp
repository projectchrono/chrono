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

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/collision/ChCollisionSystemBullet.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
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

    auto mgear = chrono_types::make_shared<ChBody>();
    mgear->SetPos(gear_center);
    my_system.Add(mgear);

    // add cylindrical lobes
    mgear->GetCollisionModel()->ClearModel();
    for (int i = 0; i< lobe_copies; ++i) {
        double phase = CH_C_2PI * ((double)i/(double)lobe_copies);
        // this is a quick shortcut from ChUtilsCreators.h, 
        // it both adds the collision shape and the visualization asset:

        utils::AddCylinderGeometry(
            mgear.get(), mysurfmaterial,                                                      //
            lobe_width * 0.5, lobe_thickness * 0.5,                                           //
            ChVector<>(lobe_primitive_rad * sin(phase), lobe_primitive_rad * cos(phase), 0),  //
            Q_from_AngAxis(CH_C_PI_2, VECT_X),  // rotate cylinder axis: from default on Y axis, to Z axis
            true);

        ////utils::AddBoxGeometry(
        ////    mgear.get(), mysurfmaterial, 
        ////    ChVector<>(lobe_width, lobe_outer_rad - lobe_inner_rad, lobe_thickness) * 0.5,
        ////    ChVector<>(0.5 * (lobe_outer_rad + lobe_inner_rad) * sin(phase),
        ////               0.5 * (lobe_outer_rad + lobe_inner_rad) * cos(phase), 0),
        ////    Q_from_AngAxis(-phase, VECT_Z),  // rotate cylinder axis: from default on Y axis, to Z axis
        ////    true);
    }

    utils::AddCylinderGeometry(mgear.get(), mysurfmaterial, lobe_inner_rad, lobe_thickness * 0.5, ChVector<>(0, 0, 0),
                               Q_from_AngAxis(CH_C_PI_2, VECT_X), true);
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
    auto mground = chrono_types::make_shared<ChBody>();
    mground->SetBodyFixed(true);
    my_system.Add(mground);

    // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.

    auto my_mesh = chrono_types::make_shared<ChMesh>();
    my_system.Add(my_mesh);

    // Create a section, i.e. thickness and material properties
    // for beams. This will be shared among some beams.

    double wire_diameter = 0.010;

	auto minertia = chrono_types::make_shared<ChInertiaCosseratSimple>();
	minertia->SetAsCircularSection(wire_diameter, 2700);  // automatically sets A etc., from width, height, density 

	auto melasticity = chrono_types::make_shared<ChElasticityCosseratSimple>();
	melasticity->SetYoungModulus(0.5e9);
	melasticity->SetGshearModulus(0.5e9 * 0.7);
    melasticity->SetAsCircularSection(wire_diameter);

	auto mdamping = chrono_types::make_shared<ChDampingCosseratLinear>();
	mdamping->SetDampingCoefficientsRe((1e-3)*ChVector<>(1, 1, 1)); 
	mdamping->SetDampingCoefficientsRk((1e-4)*ChVector<>(1, 1, 1)); //***??? -/+

	auto mplasticity = chrono_types::make_shared<ChPlasticityCosseratLumped>();
	mplasticity->n_yeld_Mx = chrono_types::make_shared<ChFunction_Ramp>(1, 0.01);
	mplasticity->n_yeld_My = chrono_types::make_shared<ChFunction_Ramp>(0.2, 0.001);
	mplasticity->n_yeld_Mz = chrono_types::make_shared<ChFunction_Ramp>(0.2, 0.001);

	auto msection = chrono_types::make_shared<ChBeamSectionCosserat>(minertia, melasticity, mplasticity, mdamping);

	msection->SetCircular(true);
	msection->SetDrawCircularRadius(wire_diameter/2.);

    // Create the surface material for the contacts; this contains information about friction etc.
    // It is a SMC (penalty) material: interpenetration might happen for low Young stiffness,
    // but unstable simulation might happen for high stiffness, requiring smaller timesteps.
    
    /*
    // option A: Hertz contact force model
    my_system.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(20e3);  // to adjust heuristically..
    mysurfmaterial->SetRestitution(0.1f);
    mysurfmaterial->SetFriction(0.2f);
	*/
    
    // Option B: Hooke force model 
    my_system.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);
    my_system.UseMaterialProperties(false);
    auto mysurfmaterial = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetKn(350); // contact normal stiffness
    mysurfmaterial->SetKt(350); // contact tangential stiffness
    mysurfmaterial->SetGn(25);   // contact normal damping
    mysurfmaterial->SetGt(25);   // contact tangential damping
    mysurfmaterial->SetFriction(0.2f);
    

    //
    // Add the EXTRUDER
    //

    auto extruder = chrono_types::make_shared<ChExtruderBeamIGA>(
            &my_system,                 // the physical system 
            my_mesh,                    // the mesh where to add the beams
            msection,                   // section for created beam
            0.015,                        // beam element length (size of discretization: the smaller, the more precise)
            ChCoordsys<>(ChVector<>(0,0,0)), // outlet coordinate system (x axis is the extrusion dir)
            0.08,                        // the extrusion speed
            1                            // the order of beams
            );

    // Enable collision for extruded beam
    extruder->SetContact( mysurfmaterial,  // the NSC material for contact surfaces
                          1.15*wire_diameter*0.5  // the radius of the collision spheres at the nodes, (enlarge 15%)
                          );


	// Do we want gravity effect on FEA elements in this demo?
	my_mesh->SetAutomaticGravity(false);

    //
    // Attach a visualization of the FEM mesh.
    //

    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    mvisualizebeamA->SetColorscaleMinMax(-0.4, 0.4);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    my_mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = chrono_types::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
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
    double lobe_inner_rad = 0.13;
    double lobe_outer_rad = 0.34;
    double lobe_thickness = 0.08;
    ChVector<> gear_centerLOW(0.3,-lobe_primitive_rad+0.01,0);
    ChVector<> gear_centerHI (0.3, lobe_primitive_rad-0.01,0);

    auto gearLOW =  CreateLobedGear (gear_centerLOW, lobe_copies, lobe_width, lobe_primitive_rad, 
                    lobe_inner_rad, lobe_outer_rad, lobe_thickness, my_system, mysurfmaterial); 

    auto mgear_motorLOW = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    mgear_motorLOW->Initialize(gearLOW, mground, ChFrame<>(gear_centerLOW));
    my_system.Add(mgear_motorLOW);

    auto mgear_speedLOW = chrono_types::make_shared<ChFunction_Const>(-0.18); // [rad/s]
    mgear_motorLOW->SetSpeedFunction(mgear_speedLOW);

    auto gearHI =  CreateLobedGear (gear_centerHI, lobe_copies, lobe_width, lobe_primitive_rad, 
                    lobe_inner_rad, lobe_outer_rad, lobe_thickness, my_system, mysurfmaterial);
    gearHI->SetRot(Q_from_AngZ(0.5*CH_C_2PI/lobe_copies)); // to phase half step respect to other gear 

    auto mgear_motorHI = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    mgear_motorHI->Initialize(gearHI, mground, ChFrame<>(gear_centerHI));
    my_system.Add(mgear_motorHI);

    auto mgear_speedHI = chrono_types::make_shared<ChFunction_Const>( 0.18); // [rad/s]
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


    // SIMULATION LOOP

    auto mkl_solver = chrono_types::make_shared<ChSolverMKL>();
    mkl_solver->LockSparsityPattern(true);
    my_system.SetSolver(mkl_solver);
            
    application.SetTimestep(0.0002);
    application.SetVideoframeSaveInterval(20);
    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();
        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1,20,20,CSYSNORM, irr::video::SColor(255,100,100,100),true);

        application.DoStep();

        bool modified = extruder->Update();    //***REMEMBER*** to do this to update the extrusion
        if (modified) {
            // A system change occurred: if using a sparse direct linear solver and if using the sparsity pattern
            // learner (enabled by default), then we must force a re-evaluation of system matrix sparsity pattern!
            mkl_solver->ForceSparsityPatternUpdate();
        }

        application.EndScene();
    }

    return 0;
}

