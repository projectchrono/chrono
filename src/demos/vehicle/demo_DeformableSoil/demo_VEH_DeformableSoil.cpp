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
// Demo code illustrating the SCM semi-empirical model for deformable soil
// =============================================================================

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;

bool output = false;
const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// Type of tire (controls both contact and visualization)
enum class TireType { CYLINDRICAL, LUGGED };
TireType tire_type = TireType::LUGGED;

// SCM grid spacing
double mesh_resolution = 0.04;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
// Here, the vehicle moves in the terrain's negative y direction!
class MySoilParams : public vehicle::SCMDeformableTerrain::SoilParametersCallback {
  public:
    virtual void Set(const ChVector<>& loc,
                     double& Bekker_Kphi,
                     double& Bekker_Kc,
                     double& Bekker_n,
                     double& Mohr_cohesion,
                     double& Mohr_friction,
                     double& Janosi_shear,
                     double& elastic_K,
                     double& damping_R) override {
        if (loc.y() > 0) {
            Bekker_Kphi = 0.2e6;
            Bekker_Kc = 0;
            Bekker_n = 1.1;
            Mohr_cohesion = 0;
            Mohr_friction = 30;
            Janosi_shear = 0.01;
            elastic_K = 4e7;
            damping_R = 3e4;
        } else {
            Bekker_Kphi = 5301e3;
            Bekker_Kc = 102e3;
            Bekker_n = 0.793;
            Mohr_cohesion = 1.3e3;
            Mohr_friction = 31.1;
            Janosi_shear = 1.2e-2;
            elastic_K = 4e8;
            damping_R = 3e4;
        }
    }
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Set world frame with Y up
    vehicle::ChWorldFrame::SetYUP();

    // Global parameter for tire:
    double tire_rad = 0.8;
    ChVector<> tire_center(0, 0.02 + tire_rad, -1.5);

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Deformable soil", core::dimension2d<u32>(1280, 720), VerticalDir::Y, false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(2.0f, 1.4f, 0.0f), core::vector3df(0, (f32)tire_rad, 0));
    application.AddLightWithShadow(core::vector3df(1.5f, 5.5f, -2.5f), core::vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));

    std::shared_ptr<ChBody> mtruss(new ChBody);
    mtruss->SetBodyFixed(true);
    my_system.Add(mtruss);

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    //
    // Create a rigid body with a mesh or a cylinder collision shape
    //

    std::shared_ptr<ChBody> mrigidbody(new ChBody);
    my_system.Add(mrigidbody);
    mrigidbody->SetMass(500);
    mrigidbody->SetInertiaXX(ChVector<>(20, 20, 20));
    mrigidbody->SetPos(tire_center + ChVector<>(0, 0.3, 0));

    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    mrigidbody->GetCollisionModel()->ClearModel();
    switch (tire_type) {
        case TireType::LUGGED: {
            auto trimesh = chrono_types::make_shared<geometry::ChTriangleMeshConnected>();
            trimesh->LoadWavefrontMesh(GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj"));

            std::shared_ptr<ChTriangleMeshShape> mrigidmesh(new ChTriangleMeshShape);
            mrigidmesh->SetMesh(trimesh);
            mrigidbody->AddAsset(mrigidmesh);

            mrigidbody->GetCollisionModel()->AddTriangleMesh(material, trimesh, false, false, VNULL, ChMatrix33<>(1),
                                                             0.01);
            break;
        }
        case TireType::CYLINDRICAL: {
            double radius = 0.5;
            double width = 0.4;
            mrigidbody->GetCollisionModel()->AddCylinder(material, radius, radius, width / 2, ChVector<>(0), Q_from_AngZ(CH_C_PI_2));
            
            auto cyl_shape = chrono_types::make_shared<ChCylinderShape>();
            cyl_shape->GetCylinderGeometry().rad = radius;
            cyl_shape->GetCylinderGeometry().p1 = ChVector<>(+width / 2, 0, 0);
            cyl_shape->GetCylinderGeometry().p2 = ChVector<>(-width / 2, 0, 0);
            mrigidbody->AddAsset(cyl_shape);

            break;
        }
    }
    mrigidbody->GetCollisionModel()->BuildModel();
    mrigidbody->SetCollide(true);

    std::shared_ptr<ChColorAsset> mcol(new ChColorAsset);
    mcol->SetColor(ChColor(0.3f, 0.3f, 0.3f));
    mrigidbody->AddAsset(mcol);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, CH_C_PI / 4.0));
    motor->Initialize(mrigidbody, mtruss, ChFrame<>(tire_center, Q_from_AngY(CH_C_PI_2)));
    my_system.Add(motor);

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMDeformableTerrain mterrain(&my_system);

    // Displace/rotate the terrain reference plane.
    // Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0.2, 0), Q_from_AngX(-CH_C_PI_2)));

    // Initialize the geometry of the soil

    // Use either a regular grid:
    double length = 6;
    double width = 2;
    mterrain.Initialize(width, length, mesh_resolution);

    // Or use a height map:
    ////mterrain.Initialize(vehicle::GetDataFile("terrain/height_maps/test64.bmp"), "test64", 1.6, 1.6, 0, 0.3);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Location-dependent soil properties
        auto my_params = chrono_types::make_shared<MySoilParams>();
        mterrain.RegisterSoilParametersCallback(my_params);
    } else {
        // Constant soil properties
        mterrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                   0,      // Bekker Kc
                                   1.1,    // Bekker n exponent
                                   0,      // Mohr cohesive limit (Pa)
                                   30,     // Mohr friction limit (degrees)
                                   0.01,   // Janosi shear coefficient (m)
                                   4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                   3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );

        // LETE sand parameters
        ////mterrain.SetSoilParameters(5301e3,  // Bekker Kphi
        ////                           102e3,   // Bekker Kc
        ////                           0.793,   // Bekker n exponent
        ////                           1.3e3,   // Mohr cohesive limit (Pa)
        ////                           31.1,    // Mohr friction limit (degrees)
        ////                           1.2e-2,  // Janosi shear coefficient (m)
        ////                           4e8,     // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
        ////                           3e4      // Damping (Pa s/m), proportional to negative vertical speed (optional)
        ////);
    }

    if (enable_bulldozing) {
        mterrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        mterrain.SetBulldozingParameters(
            55,   // angle of friction for erosion of displaced material at the border of the rut
            1,    // displaced material vs downward pressed material.
            5,    // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Optionally, enable moving patch feature (reduces number of ray casts)
    if (enable_moving_patch) {
        mterrain.AddMovingPatch(mrigidbody, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * tire_rad, 2 * tire_rad));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    ////mterrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
    mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 30000.2);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE, 0, 0.15);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE_PLASTIC, 0, 0.15);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_SINKAGE_ELASTIC, 0, 0.05);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_STEP_PLASTIC_FLOW, 0, 0.0001);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_ISLAND_ID, 0, 8);
    ////mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_IS_TOUCHED, 0, 8);
    mterrain.SetMeshWireframe(true);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    /*
        // Change the timestepper to HHT:
        my_system.SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(8);
        integrator->SetAbsTolerances(1e-05, 1.8e00);
        integrator->SetMode(ChTimestepperHHT::POSITION);
        integrator->SetModifiedNewton(true);
        integrator->SetScaling(true);
        integrator->SetVerbose(true);
    */
    /*
        my_system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    */

    application.SetTimestep(0.002);

    while (application.GetDevice()->run()) {
        if (output) {
            vehicle::TerrainForce frc = mterrain.GetContactForce(mrigidbody);
            csv << my_system.GetChTime() << frc.force << frc.moment << frc.point << std::endl;
        }

        application.BeginScene();
        application.GetActiveCamera()->setTarget(core::vector3dfCH(mrigidbody->GetPos()));
        application.DrawAll();
        application.DoStep();
        tools::drawColorbar(0, 30000, "Pressure yield [Pa]", application.GetDevice(), 1180);
        application.EndScene();

        ////mterrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
