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
//   Demo code (advanced), about
//
//     - loading an Abaqus tetahedrom mesh
//     - using it as a wheel with contacts to ground


#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLoaderUV.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_mkl/ChLcpMklSolver.h"
#include "chrono_matlab/ChMatlabEngine.h"
#include "chrono_matlab/ChLcpMatlabSolver.h"
#include "chrono_irrlicht/ChIrrApp.h"


using namespace chrono;
using namespace fea;
using namespace irr;

int main(int argc, char* argv[]) {

    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0.02+tire_rad, 0.5);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y)); // create rotated 180� on y

    double tire_w0 = tire_vel_z0/tire_rad;


    // Create a Chrono::Engine physical system
    ChSystemDEM my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"FEA contacts", core::dimension2d<u32>(1280, 720), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1, (f32)1.4, -1.2), core::vector3df(0, tire_rad, 0));
    //application.SetContactsDrawMode(irr::ChIrrTools::CONTACT_DISTANCES);

    application.AddLightWithShadow(core::vector3df(1.5, 5.5, -2.5), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.8, 0.8, 1));

    //
    // CREATE THE PHYSICAL SYSTEM
    //

    // Create the surface material, containing information
    // about friction etc.

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial (new ChMaterialSurfaceDEM);
    mysurfmaterial->SetKn(2e6);
    mysurfmaterial->SetKt(2e6);
    mysurfmaterial->SetGn(4200);
    mysurfmaterial->SetGt(4200);

    ChSharedPtr<ChMaterialSurfaceDEM> mysurfmaterial2 (new ChMaterialSurfaceDEM);
    mysurfmaterial2->SetKn(12e6);
    mysurfmaterial2->SetKt(12e6);
    mysurfmaterial2->SetGn(16000);
    mysurfmaterial2->SetGt(16000);

    // RIGID BODIES
    // Create some rigid bodies, for instance a floor:

    ChSharedPtr<ChBodyEasyBox> mfloor (new ChBodyEasyBox(2,0.2,6,2700, true));
    mfloor->SetBodyFixed(true);
    mfloor->SetMaterialSurface(mysurfmaterial);
    my_system.Add(mfloor);

    ChSharedPtr<ChTexture> mtexture(new ChTexture);
    mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mfloor->AddAsset(mtexture);


    // Create a step
    if (false) {
        ChSharedPtr<ChBodyEasyBox> mfloor_step (new ChBodyEasyBox(1,0.2,0.5,2700, true));
        mfloor_step->SetPos( ChVector<>(0,0.1,-0.2));
        mfloor_step->SetBodyFixed(true);
        mfloor_step->SetMaterialSurface(mysurfmaterial);
        my_system.Add(mfloor_step);
    }

    // Create some bent rectangular fixed slabs
    if (false) {
        for (int i=0; i<50; ++i) {
            ChSharedPtr<ChBodyEasyBox> mcube (new ChBodyEasyBox(0.25,0.2,0.25,2700, true));
            ChQuaternion<> vrot;
            vrot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);
            mcube->Move( ChCoordsys<>(VNULL,vrot) );
            vrot.Q_from_AngAxis((ChRandom()-0.5)*2*CH_C_DEG_TO_RAD*20, ChVector<>(ChRandom()-0.5,0,ChRandom()-0.5).Normalize());
            mcube->Move( ChCoordsys<>(VNULL,vrot) );
            mcube->SetPos(ChVector<>((ChRandom()-0.5)*1.8, ChRandom()*0.1, -ChRandom()*3.2+0.9));
            mcube->SetBodyFixed(true);
            mcube->SetMaterialSurface(mysurfmaterial);
            my_system.Add(mcube);
            ChSharedPtr<ChColorAsset> mcubecol(new ChColorAsset);
            mcubecol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            mcube->AddAsset(mcubecol);

        }
    }

    // Create some stones / obstacles on the ground
    if (true) {
        for (int i=0; i<150; ++i) {
            ChSharedPtr<ChBodyEasyBox> mcube (new ChBodyEasyBox(0.18,0.04,0.18,2700, true));
            ChQuaternion<> vrot;
            vrot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);
            mcube->Move( ChCoordsys<>(VNULL,vrot) );
            mcube->SetPos(ChVector<>((ChRandom()-0.5)*1.4, ChRandom()*0.2+0.05,-ChRandom()*2.6+0.2));
            mcube->SetMaterialSurface(mysurfmaterial2);
            my_system.Add(mcube);
            ChSharedPtr<ChColorAsset> mcubecol(new ChColorAsset);
            mcubecol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            mcube->AddAsset(mcubecol);
        }
    }

   
    // FINITE ELEMENT MESH
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.

    ChSharedPtr<ChMesh> my_mesh(new ChMesh);


    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters

    ChSharedPtr<ChContinuumElastic> mmaterial(new ChContinuumElastic);
    mmaterial->Set_E(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.4);
    mmaterial->Set_RayleighDampingK(0.004);
    mmaterial->Set_density(1000);


    // Load an ABAQUS .INP tetahedron mesh file from disk, defining a tetahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.

    std::vector<std::vector<ChSharedPtr<ChNodeFEAbase> > > node_sets;

    try {
        my_mesh->LoadFromAbaqusFile(GetChronoDataFile("fea/tractor_wheel.INP").c_str(), mmaterial, node_sets, tire_center, tire_alignment);
    } catch (ChException myerr) {
        GetLog() << myerr.what();
        return 0;
    }



    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceNodeCloud, so just pass 
    // all nodes to it.

    ChSharedPtr<ChContactSurfaceNodeCloud> mcontactsurf (new ChContactSurfaceNodeCloud);

    for (unsigned int i = 0; i< my_mesh->GetNnodes(); ++i)
        mcontactsurf->AddNode( my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>() );

    mcontactsurf->SetMaterialSurface(mysurfmaterial);

    my_mesh->AddContactSurface(mcontactsurf);


    // Apply initial speed and angular speed
    double speed_x0 = 0.5;
    for (unsigned int i = 0; i< my_mesh->GetNnodes(); ++i) {
        ChVector<> node_pos = my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>()->GetPos();
        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos-tire_center);
        my_mesh->GetNode(i).DynamicCastTo<ChNodeFEAxyz>()->SetPos_dt(ChVector<>(0 , 0, tire_vel_z0) +  tang_vel);
    }

    // This is necessary in order to precompute the
    // stiffness matrices for all inserted elements in mesh
    my_mesh->SetupInitial();

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);


    // Add a rim
    ChSharedPtr<ChBody> mwheel_rim(new ChBody);
    mwheel_rim->SetMass(80);
    mwheel_rim->SetInertiaXX(ChVector<>(60,60,60));
    mwheel_rim->SetPos(tire_center);
    mwheel_rim->SetRot(tire_alignment);
    mwheel_rim->SetPos_dt(ChVector<>( 0, 0, tire_vel_z0));
    mwheel_rim->SetWvel_par(ChVector<>( tire_w0, 0, 0));
    application.GetSystem()->Add(mwheel_rim);

    ChSharedPtr<ChObjShapeFile> mobjmesh(new ChObjShapeFile);
    mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    mwheel_rim->AddAsset(mobjmesh);


    // Conect rim and tire using constraints.
    // Do these constraints where the 2nd node set has been marked in the .INP file.
    int nodeset_index =1;
    for (int i=0; i< node_sets[nodeset_index].size(); ++i) {
        ChSharedPtr< ChLinkPointFrame > mlink(new ChLinkPointFrame);
        mlink->Initialize(node_sets[nodeset_index][i].DynamicCastTo<ChNodeFEAxyz>(), mwheel_rim );
        my_system.Add(mlink);
    }


    /// Create a mesh surface, for applying loads:

    ChSharedPtr<ChMeshSurface> mmeshsurf (new ChMeshSurface);
    my_mesh->AddMeshSurface(mmeshsurf);
    
      // In the .INP file there are two additional NSET nodesets, the 1st is used to mark load surface:
    mmeshsurf->AddFacesFromNodeSet(node_sets[0]);


    /// Apply load to all surfaces in the mesh surface

    ChSharedPtr<ChLoadContainer> mloadcontainer(new ChLoadContainer);
    my_system.Add(mloadcontainer);

    for (int i= 0; i< mmeshsurf->GetFacesList().size(); ++i) {
        ChSharedPtr<ChLoadableUV> aface = mmeshsurf->GetFacesList()[i];
        ChSharedPtr<ChLoad< ChLoaderPressure > >  faceload (new ChLoad< ChLoaderPressure >(aface));
        faceload->loader.SetPressure(10000); // low pressure... the tire has no ply!
        mloadcontainer->Add(faceload);
    }



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

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemesh(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 10);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh); 

    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshB(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemeshB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizemeshB->SetWireframe(true);
    my_mesh->AddAsset(mvisualizemeshB); 
/*
    ChSharedPtr<ChVisualizationFEAmesh> mvisualizemeshC(new ChVisualizationFEAmesh(*(my_mesh.get_ptr())));
    mvisualizemeshC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    mvisualizemeshC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizemeshC->SetSymbolsThickness(0.006);
    my_mesh->AddAsset(mvisualizemeshC);
 */
 

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

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    
        // Change solver to embedded MINRES
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);     
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(40);
    my_system.SetTolForce(1e-10);  

   
        // Change solver to pluggable MKL
    ChLcpMklSolver* mkl_solver_stab = new ChLcpMklSolver;
    ChLcpMklSolver* mkl_solver_speed = new ChLcpMklSolver;
    my_system.ChangeLcpSolverStab(mkl_solver_stab);
    my_system.ChangeLcpSolverSpeed(mkl_solver_speed);
    application.GetSystem()->Update();
 

    // Change type of integrator:
    my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // my_system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    if (ChSharedPtr<ChTimestepperHHT> mystepper = my_system.GetTimestepper().DynamicCastTo<ChTimestepperHHT>()) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetTolerance(1e-6);
    }


    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
