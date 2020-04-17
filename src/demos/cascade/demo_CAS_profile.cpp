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
//   Show how to use the OpenCASCADE features
//   implemented in the unit_CASCADE:
//
//   - use the ChBodyEasyCascade to create a 2D shape with proper mass & inertia
//   - use the ChBodyEasyCascade to define a 2D collision profile optimized for smooth arc contacts
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_cascade/ChBodyEasyCascade.h"
#include "chrono_cascade/ChCascadeDoc.h"
#include "chrono_cascade/ChCascadeShapeAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"

// Use the namespace of Chrono
using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::irrlicht;

// Use the main namespaces of Irlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

// Use the namespace with OpenCascade stuff
using namespace cascade;

//
// This is the program which is executed
//

int main(int argc, char* argv[]) {

    // Create a ChronoENGINE physical system: all bodies and constraints
    // will be handled by this ChSystemNSC object.
    ChSystemNSC my_system;

    // Collision tolerances. 
	// note: for 2D-2D contact problems, only the margin tolerance is used (that acts outward as the envelope)
	collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0);
	collision::ChCollisionModel::SetDefaultSuggestedMargin(0.1);

    // Contact material (shared among all collision shapes)
    auto material = chrono_types::make_shared<ChMaterialSurfaceNSC>();
	material->SetFriction(0.05f);
    

    // Create the truss:
    auto mfloor = chrono_types::make_shared<ChBody>();
    mfloor->SetBodyFixed(true);
    my_system.Add(mfloor);


	//
    // Create the Geneva wheel
	//


    // Geneva wheel geometry data:
    int nstations = 5;
    double R = 1;
    double Ri = 0.5;
    double wi = 0.1;
    double Li = 0.55;
    ChVector<> geneva_center(-0, 0, 0);
    // compute aux data:
    double beta = (CH_C_2PI/(double)nstations); // angle width of station
    double gamma = 2*(CH_C_PI_2 - beta/2); 
    double B = R*tan(beta/2);
    ChVector<> crank_center = ChVector<>(B, R, 0) + geneva_center;
     

    // Create a ChLinePath geometry that represents the 2D shape of the Geneva wheel.
    // It can be made of ChLineArc or ChLineSegment sub-lines. These must be added in clockwise
    // order, and the end of sub-item i must be coincident with beginning of sub-line i+1.
    auto mpathwheel = chrono_types::make_shared<ChLinePath>();
    
    for (int i=0; i<nstations; ++i) {
        double alpha = -i*beta; // phase of current station
        ChVector<> p1 (-B+Ri, R, 0);
        ChVector<> p2 (-wi/2, R, 0);
        ChVector<> p3 (-wi/2, R-Li, 0);
        ChVector<> p4 ( wi/2, R-Li, 0);
        ChVector<> p5 ( wi/2, R, 0);
        ChVector<> p6 ( B-Ri, R, 0);
        ChVector<> p7 ( B, R, 0);
        ChMatrix33<> mm(alpha, VECT_Z);
        p1 = mm * p1;
        p2 = mm * p2;
        p3 = mm * p3;
        p4 = mm * p4;
        p5 = mm * p5;
        p6 = mm * p6;
        p7 = mm * p7;
        ChLineSegment mseg1(p1,p2);
        ChLineSegment mseg2(p2,p3);
        ChLineArc     mseg3(ChCoordsys<>((p3+p4)*0.5),wi/2,alpha+CH_C_PI,alpha+CH_C_2PI, true);
        ChLineSegment mseg4(p4,p5);
        ChLineSegment mseg5(p5,p6);
        mpathwheel->AddSubLine(mseg1);
        mpathwheel->AddSubLine(mseg2);
        mpathwheel->AddSubLine(mseg3);
        mpathwheel->AddSubLine(mseg4);
        mpathwheel->AddSubLine(mseg5);
        double a1 = alpha+CH_C_PI;
        double a2 = alpha+CH_C_PI+gamma;
        ChLineArc marc0(ChCoordsys<>(p7), Ri, a1, a2, true); // ccw arc because concave
        mpathwheel->AddSubLine(marc0);
    }

    
    // Create the rotating Geneva wheel using the ChBodyEasyCascadeProfile  body, this will automate 
	// some useful operations:
	// - it computes a triangulation of face surrounded by profile (and holes, if any) as a visualizer asset
	// - it computes the mass and the inertia tensor given the profile
	// - it moves the COG in the computed position
	// - it adds a 2D collision profile optimized for 2D-2D collision scenarios, with smooth arc collisions. 
	// Might throw exception if some wire path not on XY plane, or not closed.

	auto mgenevawheel = chrono_types::make_shared<ChBodyEasyCascadeProfile>(
							std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> { mpathwheel }, // wire(s) containing the face, made with arcs and segments
							std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> { }, // wire(s) telling holes (empty vector here)
							0.05,		// the thickness
							1000,		// the density
							ChCascadeTriangulateTolerances(0.01,false,0.2), // triangulation tolerances for visualization, finer than default ChCascadeTriangulateTolerances()
							true, material // enable 2D collision on the profile
							);
	mgenevawheel->SetFrame_REF_to_abs(ChFrame<>(geneva_center));
    mgenevawheel->SetWvel_loc(ChVector<>(0,0,-0.08));
	my_system.Add(mgenevawheel);

	// Do you need an additional profile at a different Z depht?
	// If so, use the AddProfile() function. It also updates the mass, COG position, collision shapes, etc. 
	// Might throw exception if path not on XY plane, or not closed.
    auto mpathcam = chrono_types::make_shared<ChLinePath>();
	auto mcamline1 = chrono_types::make_shared<ChLineArc>(ChCoordsys<>(ChVector<>(0, 0, -0.10)), R * 0.3, 1.5, CH_C_PI);//CH_C_2PI, 2.5);
	auto mcamline2 = chrono_types::make_shared<ChLineSegment>(mcamline1->GetEndB(), mcamline1->GetEndA());
    mpathcam->AddSubLine(mcamline1);
	mpathcam->AddSubLine(mcamline2);
	mgenevawheel->AddProfile(	{ mpathcam }, // wire(s) containing the face, made with arcs and segments
						{},					  // wire(s) telling holes (empty vector here)
						0.09, 1000, 		  // thickness, density
						ChCascadeTriangulateTolerances(), // triangulation tolerances for visualization, default values
						true, material);    // enable 2D collision on the profile
	


	// Revolute constraint 
    auto mrevolute = chrono_types::make_shared<ChLinkLockRevolute>();
    mrevolute->Initialize(mgenevawheel, mfloor, ChCoordsys<>(geneva_center));
    my_system.Add(mrevolute);



	//
    // Create the crank:
	//

     // Create a ChLinePath geometry, and insert sub-paths in clockwise order:
    auto mpathcrankpin = chrono_types::make_shared<ChLinePath>();
    ChLineArc mpin(ChCoordsys<>(ChVector<>(-B, 0, 0)), wi/2-0.005, CH_C_2PI, 0);
    mpathcrankpin->AddSubLine(mpin);

    auto mpathcrankstopper = chrono_types::make_shared<ChLinePath>();
    ChLineArc     mstopperarc(ChCoordsys<>(ChVector<>(0, 0, 0)), Ri-0.003, CH_C_PI-gamma/2, -CH_C_PI+gamma/2);
    ChLineSegment mstopperve1(mstopperarc.GetEndB(),ChVector<>(0, 0, 0));
    ChLineSegment mstopperve2(ChVector<>(0, 0, 0), mstopperarc.GetEndA());
    mpathcrankstopper->AddSubLine(mstopperarc);
    mpathcrankstopper->AddSubLine(mstopperve1);
    mpathcrankstopper->AddSubLine(mstopperve2);

	// Use the ChBodyEasyCascadeProfile to define a 2D profile, again:

	auto mcrank = chrono_types::make_shared<ChBodyEasyCascadeProfile>(
							std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> { mpathcrankstopper,  mpathcrankpin }, // wire(s) containing the face, made with arcs and segments
							std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> { }, // wire(s) telling holes (empty vector here)
							0.05,		// the thickness
							1000,		// the density
							ChCascadeTriangulateTolerances(0.01,false,0.2), // triangulation tolerances for visualization, finer than default ChCascadeTriangulateTolerances()
							true, material // enable 2D collision on the profile
							);

	// Do you need an additional profile at a different Z depht?
	// If so, use the AddProfile() function. It also updates the mass, COG position, collision shapes, etc. 
	// Might throw exception if path not on XY plane, or not closed.
    auto mpathbackplate = chrono_types::make_shared<ChLinePath>();
    auto mbackplate = chrono_types::make_shared<ChLineArc>(ChCoordsys<>(ChVector<>(0, 0, 0.06)), Ri*1.3, CH_C_2PI, 0);
    mpathbackplate->AddSubLine(mbackplate);
	mcrank->AddProfile(	{ mpathbackplate }, // wire(s) containing the face, made with arcs and segments
						{},					// wire(s) telling holes (empty vector here)
						0.05, 1000, 		// thickness, density
						ChCascadeTriangulateTolerances(0.01,false,0.2), 
						true, material);    // enable 2D collision on the profile
	
	mcrank->SetFrame_REF_to_abs(ChFrame<>(crank_center)); // the REF is the coordinate where the path has been defined, the COG maybe elsewhere
	my_system.Add(mcrank);

	// Should you later change some geometry, do something like this:
	// mbackplate->radius = Ri * 2;
	// mcrank->UpdateCollisionAndVisualizationShapes(ChCascadeTriangulateTolerances());


    // Add a motor between crank and truss
    auto my_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    my_motor->Initialize(mcrank, mfloor, ChFrame<>(crank_center));
    my_motor->SetSpeedFunction(chrono_types::make_shared<ChFunction_Const>(CH_C_PI / 8.0));
    my_system.AddLink(my_motor);


	// 
	// The follower
	// 

	// Create a simple follower falling on the little cam

	auto mfollowerwire = chrono_types::make_shared<ChLinePath>();
	double Z_layer_2 = -0.10;
	ChLineSegment msfoll1(ChVector<>(-0.1, R*0.3+0.1, Z_layer_2), ChVector<>(-0.1, R*0.3, Z_layer_2));
	ChLineSegment msfoll2(msfoll1.GetEndB(), ChVector<>(-2*R, R*0.3, Z_layer_2));
	ChLineSegment msfoll3(msfoll2.GetEndB(), ChVector<>(-2*R, R*0.3+0.1, Z_layer_2));
	ChLineSegment msfoll4(msfoll3.GetEndB(), msfoll1.GetEndA());
	
	mfollowerwire->AddSubLine(msfoll1);
	mfollowerwire->AddSubLine(msfoll2);
	mfollowerwire->AddSubLine(msfoll3);
	mfollowerwire->AddSubLine(msfoll4);

	auto mfollower = chrono_types::make_shared<ChBodyEasyCascadeProfile>(
							std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> { mfollowerwire }, // wire(s) containing the face, made with arcs and segments
							std::vector<std::shared_ptr<::chrono::geometry::ChLinePath>> { }, // wire(s) telling holes (empty vector here)
							0.09,		// the thickness
							1000,		// the density
							ChCascadeTriangulateTolerances(0.01,false,0.2), // triangulation tolerances for visualization, finer than default ChCascadeTriangulateTolerances()
							true, material // enable 2D collision on the profile
							);
	my_system.Add(mfollower);

	// Revolute constraint 
    auto mrevolute2 = chrono_types::make_shared<ChLinkLockRevolute>();
    mrevolute2->Initialize(mfollower, mfloor, ChCoordsys<>(ChVector<>(-1.4*R, R * 0.3 + 0.05, Z_layer_2)));
    my_system.Add(mrevolute2);


	//
    // THE VISIALIZATION SYSTEM
    //


	// Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Use 2D profiles with OpenCASCADE for mass, inertia, meshing", core::dimension2d<u32>(1024, 768), false, true);

    // Easy shortcuts to add logo, camera, lights and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(core::vector3df(30, 100, 30), core::vector3df(30, -80, -30), 200, 130);
    application.AddTypicalCamera(core::vector3df(0.2f, 0.2f, -2.3f));
	application.AddLightWithShadow(core::vector3df(1.5, 5.5, -3.5), core::vector3df(0, 0, 0), 8.2, 2.2, 8.2, 40, 512,
                                   video::SColorf(0.8f, 0.8f, 0.8f));

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

	application.AddShadowAll();


    //
    // THE SOFT-REAL-TIME CYCLE, SHOWING THE SIMULATION
    //

    application.SetTimestep(0.01);

    while (application.GetDevice()->run()) {

        application.BeginScene(true, true, video::SColor(255, 140, 161, 192));

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
