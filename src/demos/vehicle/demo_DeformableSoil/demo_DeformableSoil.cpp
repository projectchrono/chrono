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

#include "chrono/geometry/ChCTriangleMeshConnected.h"
#include "chrono/lcp/ChLcpIterativeMINRES.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/DeformableTerrain.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

int main(int argc, char* argv[]) {
    // Global parameter for tire:
    double tire_rad = 0.8;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, 0.02+tire_rad, 0);
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
    application.AddTypicalCamera(core::vector3df(1.0f, 1.4f, -1.2f), core::vector3df(0, (f32)tire_rad, 0));
    application.AddLightWithShadow(core::vector3df(1.5f, 5.5f, -2.5f), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));

    std::shared_ptr<ChBody> mtruss (new ChBody);
    mtruss->SetBodyFixed(true);
    my_system.Add(mtruss);
 
    //
    // CREATE A RIGID BODY WITH A MESH
    //

    // Create also a rigid body with a rigid mesh that will be used for the cosimulation,
    // this time the ChLoadContactSurfaceMesh cannot be used as in the FEA case, so we
    // will use the ChLoadBodyMesh class:

    std::shared_ptr<ChBody> mrigidbody (new ChBody);
    my_system.Add(mrigidbody);
    mrigidbody->SetMass(200);
    mrigidbody->SetInertiaXX(ChVector<>(20,20,20));
    mrigidbody->SetPos(tire_center + ChVector<>(0,0.3,0));

    std::shared_ptr<ChTriangleMeshShape> mrigidmesh(new ChTriangleMeshShape);
    mrigidmesh->GetMesh().LoadWavefrontMesh(GetChronoDataFile("tractor_wheel.obj"));
    mrigidmesh->GetMesh().Transform(VNULL, Q_from_AngAxis(CH_C_PI, VECT_Y) );
    mrigidbody->AddAsset(mrigidmesh);

    mrigidbody->GetCollisionModel()->ClearModel();
    mrigidbody->GetCollisionModel()->AddTriangleMesh(mrigidmesh->GetMesh(), false, false, VNULL,
                                                     ChMatrix33<>(CH_C_PI, VECT_Y), 0.01);
    mrigidbody->GetCollisionModel()->BuildModel();
    mrigidbody->SetCollide(true);

    std::shared_ptr<ChColorAsset> mcol(new ChColorAsset);
    mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    mrigidbody->AddAsset(mcol);
    
    std::shared_ptr<ChLinkEngine> myengine(new ChLinkEngine);
    myengine->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_OLDHAM);
    myengine->Set_eng_mode(ChLinkEngine::ENG_MODE_SPEED);
    if (auto mfun = std::dynamic_pointer_cast<ChFunction_Const>(myengine->Get_spe_funct()) )
        mfun->Set_yconst(CH_C_PI / 4.0);
    myengine->Initialize(mrigidbody, mtruss, ChCoordsys<>(tire_center, Q_from_AngAxis(CH_C_PI_2,VECT_Y)));
    my_system.Add(myengine);

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::DeformableTerrain mterrain(&my_system);

    // Optionally, displace/tilt/rotate the terrain reference plane:
    mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, 0.5)));

    // Initialize the geometry of the soil: use either a regular grid:
     mterrain.Initialize(0.2,1.5,5,100,200);
    // or use a height map:
    //mterrain.Initialize(vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 1.6, 1.6, 0, 0.3);

    // Set the soil terramechanical parameters:
    mterrain.SetSoilParametersSCM(1.2e6,  // Bekker Kphi
                                    0,   // Bekker Kc
                                    1.1, // Bekker n exponent
                                    0,   // Mohr cohesive limit (Pa)
                                    30,  // Mohr friction limit (degrees)
                                    0.01,// Janosi shear coefficient (m)
                                    5e7  // Elastic stiffness (Pa/m), before plastic yeld, must be > Kphi 
                                    );
    mterrain.SetBulldozingFlow(true);    // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(40, // angle of frictionfor erosion of displaced material at the border of the rut
                                    1.6);// displaced material vs downward pressed material.

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    //mterrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_PRESSURE, 0, 30000.2);
    mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_SINKAGE_PLASTIC, 0, 0.15);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_SINKAGE_ELASTIC, 0, 0.05);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_STEP_PLASTIC_FLOW, 0, 0.0001);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_ISLAND_ID, 0, 8);
    //mterrain.SetPlotType(vehicle::DeformableTerrain::PLOT_IS_TOUCHED, 0, 8);

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

    application.SetTimestep(0.005);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        ChIrrTools::drawColorbar(0,30000, "Pressure yeld [Pa]", application.GetDevice(),  1180);

        application.EndScene();
    }

    return 0;
}
