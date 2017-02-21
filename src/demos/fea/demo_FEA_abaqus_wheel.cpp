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
#include "chrono/solver/ChSolverMINRES.h"

#include "chrono_fea/ChElementTetra_4.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChMeshFileLoader.h"
#include "chrono_fea/ChContactSurfaceMesh.h"
#include "chrono_fea/ChContactSurfaceNodeCloud.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"
#include "chrono_mkl/ChSolverMKL.h"
#include "chrono_irrlicht/ChIrrApp.h"


using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {

    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0.02+tire_rad, 0.5);
    ChMatrix33<> tire_alignment(Q_from_AngAxis(CH_C_PI, VECT_Y)); // create rotated 180° on y

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
    application.AddTypicalCamera(core::vector3dfCH(ChVector<>(1, 1.4, -1.2)),
                                 core::vector3dfCH(ChVector<>(0, tire_rad, 0)));
    // application.SetContactsDrawMode(irr::ChIrrTools::CONTACT_DISTANCES);

    application.AddLightWithShadow(core::vector3dfCH(ChVector<>(1.5, 5.5, -2.5)), core::vector3df(0, 0, 0), 3, 2.2, 7.2,
                                   40, 512, video::SColorf((f32)0.8, (f32)0.8, (f32)1.0));

    //
    // CREATE THE PHYSICAL SYSTEM
    //

    // Create the surface material, containing information
    // about friction etc.
    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceDEM>();
    mysurfmaterial->SetYoungModulus(10e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    auto mysurfmaterial2 = std::make_shared<ChMaterialSurfaceDEM>();
    mysurfmaterial->SetYoungModulus(30e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0);

    // RIGID BODIES
    // Create some rigid bodies, for instance a floor:
    auto mfloor = std::make_shared<ChBodyEasyBox>(2,0.2,6,2700, true);
    mfloor->SetBodyFixed(true);
    mfloor->SetMaterialSurface(mysurfmaterial);
    my_system.Add(mfloor);

    auto mtexture = std::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    mfloor->AddAsset(mtexture);


    // Create a step
    if (false) {
        auto mfloor_step = std::make_shared<ChBodyEasyBox>(1,0.2,0.5,2700, true);
        mfloor_step->SetPos( ChVector<>(0,0.1,-0.2));
        mfloor_step->SetBodyFixed(true);
        mfloor_step->SetMaterialSurface(mysurfmaterial);
        my_system.Add(mfloor_step);
    }

    // Create some bent rectangular fixed slabs
    if (false) {
        for (int i=0; i<50; ++i) {
            auto mcube = std::make_shared<ChBodyEasyBox>(0.25,0.2,0.25,2700, true);
            ChQuaternion<> vrot;
            vrot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);
            mcube->Move( ChCoordsys<>(VNULL,vrot) );
            vrot.Q_from_AngAxis((ChRandom()-0.5)*2*CH_C_DEG_TO_RAD*20, ChVector<>(ChRandom()-0.5,0,ChRandom()-0.5).Normalize());
            mcube->Move( ChCoordsys<>(VNULL,vrot) );
            mcube->SetPos(ChVector<>((ChRandom()-0.5)*1.8, ChRandom()*0.1, -ChRandom()*3.2+0.9));
            mcube->SetBodyFixed(true);
            mcube->SetMaterialSurface(mysurfmaterial);
            my_system.Add(mcube);
            auto mcubecol = std::make_shared<ChColorAsset>();
            mcubecol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            mcube->AddAsset(mcubecol);

        }
    }

    // Create some stones / obstacles on the ground
    if (true) {
        for (int i=0; i<150; ++i) {
            auto mcube = std::make_shared<ChBodyEasyBox>(0.18,0.04,0.18,2700, true);
            ChQuaternion<> vrot;
            vrot.Q_from_AngAxis(ChRandom()*CH_C_2PI, VECT_Y);
            mcube->Move( ChCoordsys<>(VNULL,vrot) );
            mcube->SetPos(ChVector<>((ChRandom()-0.5)*1.4, ChRandom()*0.2+0.05,-ChRandom()*2.6+0.2));
            mcube->SetMaterialSurface(mysurfmaterial2);
            my_system.Add(mcube);
            auto mcubecol = std::make_shared<ChColorAsset>();
            mcubecol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            mcube->AddAsset(mcubecol);
        }
    }

   
    // FINITE ELEMENT MESH
    // Create a mesh, that is a container for groups
    // of FEA elements and their referenced nodes.
    auto my_mesh = std::make_shared<ChMesh>();

    // Create a material, that must be assigned to each solid element in the mesh,
    // and set its parameters
    auto mmaterial = std::make_shared<ChContinuumElastic>();
    mmaterial->Set_E(0.016e9);  // rubber 0.01e9, steel 200e9
    mmaterial->Set_v(0.4);
    mmaterial->Set_RayleighDampingK(0.004);
    mmaterial->Set_density(1000);


    // Load an ABAQUS .INP tetahedron mesh file from disk, defining a tetahedron mesh.
    // Note that not all features of INP files are supported. Also, quadratic tetahedrons are promoted to linear.
    // This is much easier than creating all nodes and elements via C++ programming.
    // Ex. you can generate these .INP files using Abaqus or exporting from the SolidWorks simulation tool.

    std::vector<std::vector<std::shared_ptr<ChNodeFEAbase> > > node_sets;

    try {
        ChMeshFileLoader::FromAbaqusFile(my_mesh, GetChronoDataFile("fea/tractor_wheel_coarse.INP").c_str(), mmaterial,
                                         node_sets, tire_center, tire_alignment);
    } catch (ChException myerr) {
        GetLog() << myerr.what();
        return 0;
    }

    // Create the contact surface(s). 
    // In this case it is a ChContactSurfaceNodeCloud, so just pass 
    // all nodes to it.
    auto mcontactsurf = std::make_shared<ChContactSurfaceNodeCloud>();
    my_mesh->AddContactSurface(mcontactsurf);
    mcontactsurf->AddAllNodes();
    mcontactsurf->SetMaterialSurface(mysurfmaterial);

 

    // Apply initial speed and angular speed
    double speed_x0 = 0.5;
    for (unsigned int i = 0; i< my_mesh->GetNnodes(); ++i) {
        ChVector<> node_pos = std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->GetPos();
        ChVector<> tang_vel = Vcross(ChVector<>(tire_w0, 0, 0), node_pos-tire_center);
        std::dynamic_pointer_cast<ChNodeFEAxyz>(my_mesh->GetNode(i))->SetPos_dt(ChVector<>(0 , 0, tire_vel_z0) +  tang_vel);
    }

    // Remember to add the mesh to the system!
    my_system.Add(my_mesh);


    // Add a rim
    auto mwheel_rim = std::make_shared<ChBody>();
    mwheel_rim->SetMass(80);
    mwheel_rim->SetInertiaXX(ChVector<>(60,60,60));
    mwheel_rim->SetPos(tire_center);
    mwheel_rim->SetRot(tire_alignment);
    mwheel_rim->SetPos_dt(ChVector<>( 0, 0, tire_vel_z0));
    mwheel_rim->SetWvel_par(ChVector<>( tire_w0, 0, 0));
    application.GetSystem()->Add(mwheel_rim);

    auto mobjmesh = std::make_shared<ChObjShapeFile>();
    mobjmesh->SetFilename(GetChronoDataFile("fea/tractor_wheel_rim.obj"));
    mwheel_rim->AddAsset(mobjmesh);


    // Conect rim and tire using constraints.
    // Do these constraints where the 2nd node set has been marked in the .INP file.
    int nodeset_index =1;
    for (int i=0; i< node_sets[nodeset_index].size(); ++i) {
        auto mlink = std::make_shared<ChLinkPointFrame>();
        mlink->Initialize(std::dynamic_pointer_cast<ChNodeFEAxyz>(node_sets[nodeset_index][i]), mwheel_rim );
        my_system.Add(mlink);
    }


    /// Create a mesh surface, for applying loads:
    auto mmeshsurf = std::make_shared<ChMeshSurface>();
    my_mesh->AddMeshSurface(mmeshsurf);
    
      // In the .INP file there are two additional NSET nodesets, the 1st is used to mark load surface:
    mmeshsurf->AddFacesFromNodeSet(node_sets[0]);


    /// Apply load to all surfaces in the mesh surface
    auto mloadcontainer = std::make_shared<ChLoadContainer>();
    my_system.Add(mloadcontainer);

    for (int i= 0; i< mmeshsurf->GetFacesList().size(); ++i) {
        auto aface = std::shared_ptr<ChLoadableUV>(mmeshsurf->GetFacesList()[i]);
        auto faceload = std::make_shared<ChLoad< ChLoaderPressure > >(aface);
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
    auto mvisualizemesh = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
    mvisualizemesh->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_SPEED_NORM);
    mvisualizemesh->SetColorscaleMinMax(0.0, 10);
    mvisualizemesh->SetSmoothFaces(true);
    my_mesh->AddAsset(mvisualizemesh);

    /*
        auto mvisualizemeshB = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
        mvisualizemeshB->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
        mvisualizemeshB->SetWireframe(true);
        my_mesh->AddAsset(mvisualizemeshB);
    */
    /*
        auto mvisualizemeshC = std::make_shared<ChVisualizationFEAmesh>(*(my_mesh.get()));
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


    // Mark completion of system construction
    my_system.SetupInitial();


    //
    // THE SOFT-REAL-TIME CYCLE
    //



    
        // Change solver to embedded MINRES
    my_system.SetSolverType(ChSolver::Type::MINRES);     
    my_system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetMaxItersSolverSpeed(40);
    my_system.SetTolForce(1e-10);  

   
        // Change solver to pluggable MKL
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    my_system.SetSolver(mkl_solver);
    my_system.Update();


    // Change type of integrator:
    my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // fast, less precise
    // my_system.SetTimestepperType(chrono::ChTimestepper::Type::HHT);  // precise,slower, might iterate each step

    // if later you want to change integrator settings:
    
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())) {
        mystepper->SetAlpha(-0.2);
        mystepper->SetMaxiters(2);
        mystepper->SetAbsTolerances(1e-6);
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
