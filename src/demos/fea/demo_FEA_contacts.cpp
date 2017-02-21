//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//
//   Demo code about
//
//     - contacts in FEA


#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChMeshFileLoader.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChElementCableANCF.h"
#include "chrono_fea/ChBuilderBeam.h"

#include "chrono_irrlicht/ChIrrApp.h"


using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {

    // Create a Chrono::Engine physical system
    ChSystemDEM my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"FEA contacts", core::dimension2d<u32>(800, 600), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, (f32)0.6, -1));
    application.AddLightWithShadow(core::vector3df(1.5, 5.5, -2.5), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(1, 1, 1));

    application.SetContactsDrawMode(ChIrrTools::CONTACT_DISTANCES);


    //
    // CREATE THE PHYSICAL SYSTEM
    //


    //collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0); // not needed, already 0 when using ChSystemDEM
    collision::ChCollisionModel::SetDefaultSuggestedMargin(0.006); // max inside penetration - if not enough stiffness in material: troubles

    // Use this value for an outward additional layer around meshes, that can improve
    // robustness of mesh-mesh collision detection (at the cost of having unnatural inflate effect)
    double sphere_swept_thickness = 0.002;

    // Create the surface material, containing information
    // about friction etc. 
    // It is a DEM-p (penalty) material that we will assign to 
    // all surfaces that might generate contacts.

    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceDEM>();
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0); 

    // Create a floor:

    bool do_mesh_collision_floor = false;

    ChTriangleMeshConnected mmeshbox;
    mmeshbox.LoadWavefrontMesh(GetChronoDataFile("cube.obj"),true,true);

    if(do_mesh_collision_floor) {

        // floor as a triangle mesh surface:
        auto mfloor = std::make_shared<ChBody>();
        mfloor->SetPos(ChVector<>(0, -1, 0));
        mfloor->SetBodyFixed(true);
        mfloor->SetMaterialSurface(mysurfmaterial);
        my_system.Add(mfloor);

        mfloor->GetCollisionModel()->ClearModel();
        mfloor->GetCollisionModel()->AddTriangleMesh(mmeshbox,false, false, VNULL, ChMatrix33<>(1), sphere_swept_thickness);
        mfloor->GetCollisionModel()->BuildModel();
        mfloor->SetCollide(true);

        auto masset_meshbox = std::make_shared<ChTriangleMeshShape>();
        masset_meshbox->SetMesh(mmeshbox);
        mfloor->AddAsset(masset_meshbox);

        auto masset_texture = std::make_shared<ChTexture>();
        masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
        mfloor->AddAsset(masset_texture);
        
    }
    else {
        // floor as a simple collision primitive:

        auto mfloor = std::make_shared<ChBodyEasyBox>(2, 0.1, 2, 2700, true);
        mfloor->SetBodyFixed(true);
        mfloor->SetMaterialSurface(mysurfmaterial);
        my_system.Add(mfloor);

        auto masset_texture = std::make_shared<ChTexture>();
        masset_texture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
        mfloor->AddAsset(masset_texture);
    }

    

    // two falling objects:

    auto mcube = std::make_shared<ChBodyEasyBox>(0.1, 0.1, 0.1, 2700, true);
    mcube->SetPos(ChVector<>(0.6,0.5,0.6));
    mcube->SetMaterialSurface(mysurfmaterial);
    my_system.Add(mcube);

    auto msphere = std::make_shared<ChBodyEasySphere>(0.1, 2700, true);
    msphere->SetPos(ChVector<>(0.8,0.5,0.6));
    msphere->SetMaterialSurface(mysurfmaterial);
    my_system.Add(msphere);


    //
    // Example 1: tetrahedrons, with collisions
    // 

    // Create a mesh. We will use it for tetahedrons.

    auto my_mesh = std::make_shared<ChMesh>();

    // 1) a FEA tetahedron(s):

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto mmaterial = std::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.01e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.3);
    mmaterial->Set_RayleighDampingK(0.003);
    mmaterial->Set_density(1000);

    if (false) {
        for (int k=0; k<3; ++k)
        for (int j=0; j<3; ++j)
        for (int i=0; i<3; ++i) {
            // Creates the nodes for the tetahedron
            ChVector<> offset(j*0.21, i*0.21, k*0.21);
            auto mnode1 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0.1, 0) + offset);
            auto mnode2 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0.1, 0.2) + offset);
            auto mnode3 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0, 0.3, 0) + offset);
            auto mnode4 = std::make_shared<ChNodeFEAxyz>(ChVector<>(0.2, 0.1, 0) + offset);

            my_mesh->AddNode(mnode1);
            my_mesh->AddNode(mnode2);
            my_mesh->AddNode(mnode3);
            my_mesh->AddNode(mnode4);

            auto melement1 = std::make_shared<ChElementTetra_4>();
            melement1->SetNodes(mnode1,
                                mnode2, 
                                mnode3, 
                                mnode4);
            melement1->SetMaterial(mmaterial);

            my_mesh->AddElement(melement1);
        }
    }

    if (true) {
        for (int i= 0; i<4; ++i) {
            try
            {
            ChCoordsys<> cdown(ChVector<>(0,-0.4,0));
            ChCoordsys<> crot(VNULL, Q_from_AngAxis(CH_C_2PI * ChRandom(), VECT_Y) * Q_from_AngAxis(CH_C_PI_2, VECT_X));
            ChCoordsys<> cydisp(ChVector<>(-0.3 ,0.1+i*0.1, -0.3));
            ChCoordsys<> ctot = cdown >> crot >> cydisp;
            ChMatrix33<> mrot(ctot.rot);
            ChMeshFileLoader::FromTetGenFile(my_mesh, GetChronoDataFile("fea/beam.node").c_str(),
                                             GetChronoDataFile("fea/beam.ele").c_str(), mmaterial, ctot.pos, mrot);
            }
            catch (ChException myerr) {
                    GetLog() << myerr.what();
                    return 0;
            }
        }
    }


    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collsions.

    auto mcontactsurf = std::make_shared<ChContactSurfaceMesh>();
    my_mesh->AddContactSurface(mcontactsurf);

    mcontactsurf->AddFacesFromBoundary(sphere_swept_thickness); // do this after my_mesh->AddContactSurface

    mcontactsurf->SetMaterialSurface(mysurfmaterial); // use the DEM penalty contacts

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);


    //
    // Example 2: beams, with collisions
    // 

    // Create a mesh. We will use it for beams only. 

    auto my_mesh_beams = std::make_shared<ChMesh>();

    // 2) an ANCF cable:

    auto msection_cable2 = std::make_shared<ChBeamSectionCable>();
	msection_cable2->SetDiameter(0.05);
	msection_cable2->SetYoungModulus (0.01e9);
	msection_cable2->SetBeamRaleyghDamping(0.05);

	ChBuilderBeamANCF builder;

	builder.BuildBeam(	my_mesh_beams,	// the mesh where to put the created nodes and elements 
						msection_cable2,// the ChBeamSectionCable to use for the ChElementCableANCF elements
						10,				// the number of ChElementCableANCF to create
						ChVector<>(0, 0.1, -0.1),		// the 'A' point in space (beginning of beam)
						ChVector<>(0.5, 0.13, -0.1));	// the 'B' point in space (end of beam)

    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceNodeCloud, so just pass 
    // all nodes to it.

    auto mcontactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
    my_mesh_beams->AddContactSurface(mcontactcloud);

    mcontactcloud->AddAllNodes(0.025); // use larger point size to match beam section radius

    mcontactcloud->SetMaterialSurface(mysurfmaterial);

    
    // Remember to add the mesh to the system!
    my_system.Add(my_mesh_beams);


    
    //
    // Optional...  visualization
    //

    // ==Asset== attach a visualization of the FEM mesh.
    // This will automatically update a triangle mesh (a ChTriangleMeshShape
    // asset that is internally managed) by setting  proper
    // coordinates and vertex colours as in the FEM elements.
    // Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    // postprocessor that can handle a coloured ChTriangleMeshShape).
    // Do not forget AddAsset() at the end!

    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh); 

    auto mvisualizemeshcoll = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemeshcoll->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_CONTACTSURFACES);
    mvisualizemeshcoll->SetWireframe(true);
    mvisualizemeshcoll->SetDefaultMeshColor(ChColor(1,0.5,0));
    my_mesh->AddAsset(mvisualizemeshcoll); 

    auto mvisualizemeshbeam = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh_beams.get()));
    mvisualizemeshbeam->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemeshbeam->SetColorscaleMinMax(0.0, 5.50);
    mvisualizemeshbeam->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemeshbeam);

    auto mvisualizemeshbeamnodes = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh_beams.get()));
    mvisualizemeshbeamnodes->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshbeamnodes->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshbeamnodes->SetSymbolsThickness(0.008);
    my_mesh->AddAsset(mvisualizemeshbeamnodes);
 
 

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    // in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
    // If you need a finer control on which item really needs a visualization proxy in
    // Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    // that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();


    // Mark completion of system construction
    my_system.SetupInitial();


    //
    // THE SOFT-REAL-TIME CYCLE
    //

    my_system.SetSolverType(ChSolver::Type::MINRES);     
    my_system.SetSolverWarmStarting(true);
    my_system.SetMaxItersSolverSpeed(40);
    my_system.SetTolForce(1e-10);
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  

    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
