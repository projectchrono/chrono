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
//     - apply a load to the mesh using an external tool, 
//       say CFD or SPH (here simulated as a function in this .cpp file)
//       that is perform a cosimulation.


#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/geometry/ChCTriangleMeshConnected.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_vehicle/terrain/DeformableTerrain.h"

using namespace chrono;
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
    ChIrrApp application(&my_system, L"Deformable soil", core::dimension2d<u32>(1280, 720), false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(1, (f32)1.4, -1.2), core::vector3df(0, tire_rad, 0));
    application.AddLightWithShadow(core::vector3df(1.5, 5.5, -2.5), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.8, 0.8, 1));

 
 
    //
    // CREATE A RIGID BODY WITH A MESH
    //

    // Create also a rigid body with a rigid mesh that will be used for the cosimulation,
    // this time the ChLoadContactSurfaceMesh cannot be used as in the FEA case, so we
    // will use the ChLoadBodyMesh class:

    ChSharedPtr<ChBody> mrigidbody (new ChBody);
    my_system.Add(mrigidbody);
    mrigidbody->SetMass(200);
    mrigidbody->SetInertiaXX(ChVector<>(20,20,20));
    mrigidbody->SetPos(tire_center + ChVector<>(0,1,0));

    ChSharedPtr<ChTriangleMeshShape> mrigidmesh(new ChTriangleMeshShape);
    mrigidmesh->GetMesh().LoadWavefrontMesh(GetChronoDataFile("tractor_wheel.obj"));
    mrigidmesh->GetMesh().Transform(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y) );
    mrigidbody->AddAsset(mrigidmesh);

    mrigidbody->GetCollisionModel()->ClearModel();
    mrigidbody->GetCollisionModel()->AddTriangleMesh(mrigidmesh->GetMesh(),false, false, VNULL, ChMatrix33<>(CH_C_PI, VECT_Y),0);
    mrigidbody->GetCollisionModel()->BuildModel();
    mrigidbody->SetCollide(true);

    ChSharedPtr<ChColorAsset> mcol(new ChColorAsset);
    mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    mrigidbody->AddAsset(mcol);


    //
    // THE DEFORMABLE TERRAIN
    //

    ChSharedPtr<vehicle::DeformableTerrain> mterrain (new vehicle::DeformableTerrain(&my_system));
    mterrain->Initialize(0.2,1,1,25,25);
    my_system.Add(mterrain);


    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();


    // ==IMPORTANT!== Mark completion of system construction
    my_system.SetupInitial();


    //
    // THE SOFT-REAL-TIME CYCLE
    //

 /*   
        // Change solver to embedded MINRES
        // NOTE! it is strongly advised that you compile the optional MKL module 
        // if you need higher precision, and switch to its MKL solver - see demos for FEA & MKL.
    my_system.SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);     
    my_system.SetIterLCPwarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    my_system.SetIterLCPmaxItersSpeed(40);
    my_system.SetTolForce(1e-10);  
  */  

    // Change type of integrator:
    //my_system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // fast, less precise


    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();



        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1, 20, 20,
                                 ChCoordsys<>(VNULL, CH_C_PI_2, VECT_X), video::SColor(50, 90, 90, 90), true);

        application.EndScene();
    }

    return 0;
}
