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
//     - using the SCM semi-empirical model for deformable soil
//     - using a deformable tire  

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/solver/ChSolverMINRES.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChSystemDEM.h"

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/DeformableTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"
#include "chrono_mkl/ChSolverMKL.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;

using namespace irr;

int main(int argc, char* argv[]) {
    // Global parameter for tire:
    double tire_rad = 0.5;
    double tire_vel_z0 = -3;
    ChVector<> tire_center(0, tire_rad, 0);

    double tire_w0 = tire_vel_z0/tire_rad;

    // Create a Chrono::Engine physical system
    ChSystemDEM my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Deformable soil and deformable tire", core::dimension2d<u32>(1280, 720), false, true);

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
    // CREATE A DEFORMABLE TIRE
    //
 
    // the rim: 

    std::shared_ptr<ChBody> mrim (new ChBody);
    my_system.Add(mrim);
    mrim->SetMass(100);
    mrim->SetInertiaXX(ChVector<>(2,2,2));
    mrim->SetPos(tire_center + ChVector<>(0,0.2,0));
    mrim->SetRot(Q_from_AngAxis(CH_C_PI_2, VECT_Z));

    // the tire:

    std::shared_ptr<ChReissnerTire> tire_reissner;
    tire_reissner = std::make_shared<ReissnerTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_ReissnerTire.json"));
    tire_reissner->EnablePressure(false);
    tire_reissner->EnableContact(true);
    tire_reissner->SetContactSurfaceType(ChDeformableTire::TRIANGLE_MESH);
    tire_reissner->EnableRimConnection(true);
    tire_reissner->Initialize(mrim, LEFT);
    tire_reissner->SetVisualizationType(VisualizationType::MESH);

    // the engine that rotates the rim:

    std::shared_ptr<ChLinkEngine> myengine(new ChLinkEngine);
    myengine->Set_shaft_mode(ChLinkEngine::ENG_SHAFT_OLDHAM);
    myengine->Set_eng_mode(ChLinkEngine::ENG_MODE_ROTATION);
    myengine->Set_rot_funct( std::make_shared<ChFunction_Ramp>(0, CH_C_PI / 4.0)); // CH_C_PI / 4.0) ); // phase, speed
    myengine->Initialize(mrim, mtruss, ChCoordsys<>(tire_center, Q_from_AngAxis(CH_C_PI_2,VECT_Y)));
    my_system.Add(myengine);


    //
    // THE DEFORMABLE TERRAIN
    //
    
    // Create the 'deformable terrain' object
    vehicle::DeformableTerrain mterrain(&my_system);

    // Optionally, displace/tilt/rotate the terrain reference plane:
    mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, 0.3)));

    // Initialize the geometry of the soil: use either a regular grid:
     mterrain.Initialize(0.2,1.5,5,20,60);
    // or use a height map:
    //mterrain.Initialize(vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 1.6, 1.6, 0, 0.3);

    // Set the soil terramechanical parameters:
    mterrain.SetSoilParametersSCM(1.2e6,  // Bekker Kphi
                                    0,   // Bekker Kc
                                    1.1, // Bekker n exponent
                                    0,   // Mohr cohesive limit (Pa)
                                    30,  // Mohr friction limit (degrees)
                                    0.01,// Janosi shear coefficient (m)
                                    5e7, // Elastic stiffness (Pa/m), before plastic yeld, must be > Kphi
                                    2e4  // Damping (Pa s/m), proportional to negative vertical speed (optional)
                                    );
    mterrain.SetBulldozingFlow(true);    // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(55, // angle of friction for erosion of displaced material at the border of the rut
                                    0.8, // displaced material vs downward pressed material.
                                    5,   // number of erosion refinements per timestep
                                    10); // number of concentric vertex selections subject to erosion
    // Turn on the automatic level of detail refinement, so a coarse terrain mesh
    // is automatically improved by adding more points under the wheel contact patch:
    mterrain.SetAutomaticRefinement(true);
    mterrain.SetAutomaticRefinementResolution(0.02);

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
    mterrain.GetMesh()->SetWireframe(true);
    


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

    // change the solver to MKL: 
    GetLog() << "Using MKL solver\n";
    auto mkl_solver = std::make_shared<ChSolverMKL<>>();
    mkl_solver->SetSparsityPatternLock(true);
    my_system.SetSolver(mkl_solver);
    
    
    // Change the timestepper to HHT: 
    my_system.SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxiters(8);
    integrator->SetAbsTolerances(5e-05, 1.8e00);
    integrator->SetMode(ChTimestepperHHT::POSITION);
    integrator->SetModifiedNewton(false);
    integrator->SetScaling(true);
    integrator->SetVerbose(true);
    
    

    
    application.SetTimestep(0.002);

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        application.DoStep();

        ChIrrTools::drawColorbar(0,30000, "Pressure yeld [Pa]", application.GetDevice(),  1180);

        application.EndScene();
    }

    return 0;
}
