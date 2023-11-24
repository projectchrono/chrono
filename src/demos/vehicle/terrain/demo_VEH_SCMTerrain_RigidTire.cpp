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
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#ifdef CHRONO_COLLISION
    #include "chrono/collision/multicore/ChCollisionSystemMulticore.h"
#endif
#ifdef CHRONO_POSTPROCESS
    #include "chrono_postprocess/ChBlender.h"
#endif
#ifdef CHRONO_IRRLICHT
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"
using namespace chrono::irrlicht;
#endif
#ifdef CHRONO_VSG
    #include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemVSG.h"
using namespace chrono::vsg3d;
#endif

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// =============================================================================

// Run-time visualization system (IRRLICHT or VSG)
ChVisualSystem::Type vis_type = ChVisualSystem::Type::VSG;

bool output = false;
bool blender_output = false;
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
class MySoilParams : public vehicle::SCMTerrain::SoilParametersCallback {
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
    auto collsys_type = ChCollisionSystem::Type::BULLET;
    ChSystemSMC sys;
    sys.SetNumThreads(4, 8, 1);
    switch (collsys_type) {
        case ChCollisionSystem::Type::BULLET: {
            auto collsys = chrono_types::make_shared<ChCollisionSystemBullet>();
            sys.SetCollisionSystem(collsys);
            break;
        }
        case ChCollisionSystem::Type::MULTICORE: {
#ifdef CHRONO_COLLISION
            auto collsys = chrono_types::make_shared<ChCollisionSystemMulticore>();
            collsys->SetBroadphaseGridResolution(ChVector<int>(20, 20, 10));
            sys.SetCollisionSystem(collsys);
#endif
            break;
        }
    }

    auto mtruss = chrono_types::make_shared<ChBody>();
    mtruss->SetBodyFixed(true);
    sys.Add(mtruss);

    // Initialize output
    if (output || blender_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    //
    // Create a rigid body with a mesh or a cylinder collision shape
    //

    auto wheel = chrono_types::make_shared<ChBody>();
    sys.Add(wheel);
    wheel->SetMass(500);
    wheel->SetInertiaXX(ChVector<>(20, 20, 20));
    wheel->SetPos(tire_center + ChVector<>(0, 0.3, 0));

    auto material = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    switch (tire_type) {
        case TireType::LUGGED: {
            auto trimesh = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(
                GetChronoDataFile("models/tractor_wheel/tractor_wheel.obj"));

            auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
            vis_shape->SetMesh(trimesh);
            vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            wheel->AddVisualShape(vis_shape);

            auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(material, trimesh, false, false, 0.01);
            wheel->AddCollisionShape(ct_shape, ChFrame<>(VNULL, ChMatrix33<>(1)));
            break;
        }
        case TireType::CYLINDRICAL: {
            double radius = 0.5;
            double width = 0.4;
            auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(material, radius, width);
            wheel->AddCollisionShape(ct_shape, ChFrame<>(ChVector<>(0), Q_from_AngY(CH_C_PI_2)));

            auto vis_shape = chrono_types::make_shared<ChVisualShapeCylinder>(radius, width);
            vis_shape->SetColor(ChColor(0.3f, 0.3f, 0.3f));
            wheel->AddVisualShape(vis_shape, ChFrame<>(VNULL, Q_from_AngY(CH_C_PI_2)));

            break;
        }
    }
    wheel->SetCollide(true);

    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    motor->SetAngleFunction(chrono_types::make_shared<ChFunction_Ramp>(0, CH_C_PI / 4.0));
    motor->Initialize(wheel, mtruss, ChFrame<>(tire_center, Q_from_AngY(CH_C_PI_2)));
    sys.Add(motor);

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(-CH_C_PI_2)));

    // Initialize the geometry of the soil

    // Use either a regular grid:
    double length = 6;
    double width = 2;
    mterrain.Initialize(width, length, mesh_resolution);

    // Or use a height map:
    ////mterrain.Initialize(vehicle::GetDataFile("terrain/height_maps/test64.bmp"), width, length, 0, 0.5, mesh_resolution);

    // Or use a mesh:
    ////mterrain.Initialize(vehicle::GetDataFile("terrain/meshes/test_terrain_irregular.obj"), mesh_resolution);

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
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // Optionally, enable moving patch feature (reduces number of ray casts)
    if (enable_moving_patch) {
        mterrain.AddMovingPatch(wheel, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * tire_rad, 2 * tire_rad));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    ////mterrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 30000.2);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE_PLASTIC, 0, 0.15);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE_ELASTIC, 0, 0.05);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_STEP_PLASTIC_FLOW, 0, 0.0001);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_ISLAND_ID, 0, 8);
    ////mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_IS_TOUCHED, 0, 8);
    mterrain.SetMeshWireframe(true);

    // Create the run-time visualization system
#ifndef CHRONO_IRRLICHT
    if (vis_type == ChVisualSystem::Type::IRRLICHT)
        vis_type = ChVisualSystem::Type::VSG;
#endif
#ifndef CHRONO_VSG
    if (vis_type == ChVisualSystem::Type::VSG)
        vis_type = ChVisualSystem::Type::IRRLICHT;
#endif

    std::shared_ptr<ChVisualSystem> vis;
    switch (vis_type) {
        case ChVisualSystem::Type::IRRLICHT: {
#ifdef CHRONO_IRRLICHT
            auto vis_irr = chrono_types::make_shared<ChVisualSystemIrrlicht>();
            vis_irr->SetWindowSize(800, 600);
            vis_irr->SetWindowTitle("SCM deformable terrain");
            vis_irr->Initialize();
            vis_irr->AddLogo();
            vis_irr->AddSkyBox();
            vis_irr->AddLightDirectional();
            vis_irr->AddCamera(ChVector<>(2.0, 1.4, 0.0), ChVector<>(0, tire_rad, 0));
            vis_irr->AttachSystem(&sys);

            vis = vis_irr;
#endif
            break;
        }
        default:
        case ChVisualSystem::Type::VSG: {
#ifdef CHRONO_VSG
            auto vis_vsg = chrono_types::make_shared<ChVisualSystemVSG>();
            vis_vsg->AttachSystem(&sys);
            vis_vsg->SetWindowTitle("SCM deformable terrain");
            vis_vsg->AddCamera(ChVector<>(3.0, 2.0, 0.0), ChVector<>(0, tire_rad, 0));
            vis_vsg->SetWindowSize(ChVector2<int>(800, 600));
            vis_vsg->SetWindowPosition(ChVector2<int>(100, 100));
            vis_vsg->SetClearColor(ChColor(0.8f, 0.85f, 0.9f));
            vis_vsg->SetUseSkyBox(true);
            vis_vsg->SetCameraVertical(CameraVerticalDir::Y);
            vis_vsg->SetCameraAngleDeg(40.0);
            vis_vsg->SetLightIntensity(1.0f);
            vis_vsg->SetLightDirection(1.5 * CH_C_PI_2, CH_C_PI_4);
            vis_vsg->Initialize();

            vis = vis_vsg;
#endif
            break;
        }
    }

    // Create the Blender exporter
#ifdef CHRONO_POSTPROCESS
    postprocess::ChBlender blender_exporter(&sys);
    if (blender_output) {
        blender_exporter.SetBasePath(out_dir);
        blender_exporter.SetCamera(ChVector<>(2.0, 1.4, 3.0), ChVector<>(0, tire_rad, 0), 50);
        blender_exporter.AddAll();
        blender_exporter.ExportScript();
    }
#endif

    //
    // THE SOFT-REAL-TIME CYCLE
    //
    /*
        // Change the timestepper to HHT:
        sys.SetTimestepperType(ChTimestepper::Type::HHT);
        auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
        integrator->SetAlpha(-0.2);
        integrator->SetMaxiters(8);
        integrator->SetAbsTolerances(1e-1, 10);
        integrator->SetModifiedNewton(true);
        integrator->SetVerbose(true);
    */
    /*
        sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);
    */

    while (vis->Run()) {
        double time = sys.GetChTime();
        if (output) {
            ChVector<> force;
            ChVector<> torque;
            mterrain.GetContactForceBody(wheel, force, torque);
            csv << time << force << torque << std::endl;
        }

#ifdef CHRONO_POSTPROCESS
        if (blender_output) {
            blender_exporter.ExportData();
        }
#endif

        ////std::cout << "\nTime: " << time << std::endl;
        ////std::cout << "Wheel pos: " << wheel->GetPos() << std::endl;
        ////std::cout << "Wheel rot: " << wheel->GetRot() << std::endl;

        vis->BeginScene();
        vis->SetCameraTarget(wheel->GetPos());
        vis->Render();
        ////tools::drawColorbar(vis.get(), 0, 30000, "Pressure yield [Pa]", 1180);
        vis->EndScene();

        sys.DoStepDynamics(0.002);
        ////mterrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
