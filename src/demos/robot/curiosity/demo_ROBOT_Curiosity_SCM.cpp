// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jason Zhou
// =============================================================================
//
// Demo to show Curiosity Rovering across obstacles with symmetric arrangement
// on SCM terrain
//
// =============================================================================

#include "chrono_models/robot/curiosity/Curiosity.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMDeformableTerrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include <chrono>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::curiosity;

using namespace irr;

bool output = false;
const std::string out_dir = GetChronoOutputPath() + "SCM_DEF_SOIL";

// SCM grid spacing
double mesh_resolution = 0.02;

// Enable/disable bulldozing effects
bool enable_bulldozing = true;

// Enable/disable moving patch feature
bool enable_moving_patch = true;

// If true, use provided callback to change soil properties based on location
bool var_params = true;

// Specify rover chassis type
// The options are Scarecrow and FullRover
CuriosityChassisType chassis_type = CuriosityChassisType::FullRover;

// Specify rover wheel type
// The options are RealWheel, SimpleWheel, and CylWheel
CuriosityWheelType wheel_type = CuriosityWheelType::RealWheel;

// Simulation time step
double time_step = 1e-3;

// -----------------------------------------------------------------------------

// Custom callback for setting location-dependent soil properties.
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
        Bekker_Kphi = 0.82e6;
        Bekker_Kc = 0.14e4;
        Bekker_n = 1.0;
        Mohr_cohesion = 0.017e4;
        Mohr_friction = 35.0;
        Janosi_shear = 1.78e-2;
        elastic_K = 2e8;
        damping_R = 3e4;
    }
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&sys, L"Curiosity Obstacle Crossing on SCM", core::dimension2d<u32>(1800, 1000),
                         VerticalDir::Y, false, true);
    application.AddLogo();
    application.AddSkyBox();
    application.AddTypicalLights();
    application.AddCamera(core::vector3df(2.0f, 1.4f, 0.0f), core::vector3df(0, (f32)wheel_range, 0));
    application.AddLightWithShadow(core::vector3df(-5.0f, 8.0f, -0.5f), core::vector3df(-1.0, 0, 0), 100, 1, 35, 85,
                                   512, video::SColorf(0.8f, 0.8f, 1.0f));
    application.SetTimestep(time_step);

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    // Curiosity rover initial position and orientation
    ChVector<double> body_pos(-5, -0.2, 0);
    ChQuaternion<> body_rot = Q_from_AngX(-CH_C_PI / 2);

    // Create a Curiosity rover
    Curiosity rover(&sys, chassis_type, wheel_type);

    // Create a CuriosityDriver to command the rover
    auto driver = chrono_types::make_shared<CuriositySpeedDriver>(1.0, CH_C_PI);
    rover.SetDriver(driver);
    rover.Initialize(ChFrame<>(body_pos, body_rot));

    std::shared_ptr<ChBodyAuxRef> rock_1;
    std::shared_ptr<ChBodyAuxRef> rock_2;
    std::shared_ptr<ChBodyAuxRef> rock_3;
    std::shared_ptr<ChBodyAuxRef> rock_4;
    std::shared_ptr<ChBodyAuxRef> rock_5;
    std::shared_ptr<ChBodyAuxRef> rock_6;

    // create default SMC materials for the obstacles
    std::shared_ptr<ChMaterialSurface> rockSufaceMaterial = ChMaterialSurface::DefaultMaterial(sys.GetContactMethod());

    for (int i = 0; i < 2; i++) {
        // Create a rock
        auto rock_1_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string rock1_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        double scale_ratio = 0.8;
        rock_1_mmesh->LoadWavefrontMesh(rock1_obj_path, false, true);
        rock_1_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_1_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_1_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock1_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock1_rot = ChQuaternion<>(1, 0, 0, 0);
        ChVector<> rock1_pos;
        if (i == 0) {
            rock1_pos = ChVector<>(-2.5, -0.3, -1.0);
        } else {
            rock1_pos = ChVector<>(-2.5, -0.3, 1.0);
        }

        rock1_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock1_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock1_Body->SetInertiaXX(mdensity * principal_I);

        rock1_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock1_pos), ChQuaternion<>(rock1_rot)));
        sys.Add(rock1_Body);

        rock1_Body->SetBodyFixed(false);
        rock1_Body->GetCollisionModel()->ClearModel();
        rock1_Body->GetCollisionModel()->AddTriangleMesh(rockSufaceMaterial, rock_1_mmesh, false, false, VNULL,
                                                         ChMatrix33<>(1), 0.005);
        rock1_Body->GetCollisionModel()->BuildModel();
        rock1_Body->SetCollide(true);

        auto rock1_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        rock1_mesh->SetMesh(rock_1_mmesh);
        rock1_mesh->SetBackfaceCull(true);
        rock1_Body->AddAsset(rock1_mesh);

        if (i == 0) {
            rock_1 = rock1_Body;
        } else {
            rock_2 = rock1_Body;
        }
    }

    for (int i = 0; i < 2; i++) {
        // Create a rock
        auto rock_2_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string rock2_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock1.obj");
        double scale_ratio = 0.45;
        rock_2_mmesh->LoadWavefrontMesh(rock2_obj_path, false, true);
        rock_2_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_2_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_2_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock2_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock2_rot = ChQuaternion<>(1, 0, 0, 0);
        ChVector<> rock2_pos;
        if (i == 0) {
            rock2_pos = ChVector<>(-1.0, -0.3, -1.0);
        } else {
            rock2_pos = ChVector<>(-1.0, -0.3, 1.0);
        }

        rock2_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock2_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock2_Body->SetInertiaXX(mdensity * principal_I);

        rock2_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock2_pos), ChQuaternion<>(rock2_rot)));
        sys.Add(rock2_Body);

        rock2_Body->SetBodyFixed(false);
        rock2_Body->GetCollisionModel()->ClearModel();
        rock2_Body->GetCollisionModel()->AddTriangleMesh(rockSufaceMaterial, rock_2_mmesh, false, false, VNULL,
                                                         ChMatrix33<>(1), 0.005);
        rock2_Body->GetCollisionModel()->BuildModel();
        rock2_Body->SetCollide(true);

        auto rock2_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        rock2_mesh->SetMesh(rock_2_mmesh);
        rock2_mesh->SetBackfaceCull(true);
        rock2_Body->AddAsset(rock2_mesh);

        if (i == 0) {
            rock_3 = rock2_Body;
        } else {
            rock_4 = rock2_Body;
        }
    }

    for (int i = 0; i < 2; i++) {
        // Create a rock
        auto rock_3_mmesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        std::string rock3_obj_path = GetChronoDataFile("robot/curiosity/rocks/rock3.obj");
        double scale_ratio = 0.45;
        rock_3_mmesh->LoadWavefrontMesh(rock3_obj_path, false, true);
        rock_3_mmesh->Transform(ChVector<>(0, 0, 0), ChMatrix33<>(scale_ratio));  // scale to a different size
        rock_3_mmesh->RepairDuplicateVertexes(1e-9);                              // if meshes are not watertight

        // compute mass inertia from mesh
        double mmass;
        ChVector<> mcog;
        ChMatrix33<> minertia;
        double mdensity = 8000;  // paramsH->bodyDensity;
        rock_3_mmesh->ComputeMassProperties(true, mmass, mcog, minertia);
        ChMatrix33<> principal_inertia_rot;
        ChVector<> principal_I;
        ChInertiaUtils::PrincipalInertia(minertia, principal_I, principal_inertia_rot);

        // set the abs orientation, position and velocity
        auto rock3_Body = chrono_types::make_shared<ChBodyAuxRef>();
        ChQuaternion<> rock3_rot = ChQuaternion<>(1, 0, 0, 0);
        ChVector<> rock3_pos;
        if (i == 0) {
            rock3_pos = ChVector<>(0.5, -0.3, -1.0);
        } else {
            rock3_pos = ChVector<>(0.5, -0.3, 1.0);
        }

        rock3_Body->SetFrame_COG_to_REF(ChFrame<>(mcog, principal_inertia_rot));

        rock3_Body->SetMass(mmass * mdensity);  // mmass * mdensity
        rock3_Body->SetInertiaXX(mdensity * principal_I);

        rock3_Body->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(rock3_pos), ChQuaternion<>(rock3_rot)));
        sys.Add(rock3_Body);

        rock3_Body->SetBodyFixed(false);
        rock3_Body->GetCollisionModel()->ClearModel();
        rock3_Body->GetCollisionModel()->AddTriangleMesh(rockSufaceMaterial, rock_3_mmesh, false, false, VNULL,
                                                         ChMatrix33<>(1), 0.005);
        rock3_Body->GetCollisionModel()->BuildModel();
        rock3_Body->SetCollide(true);

        auto rock3_mesh = chrono_types::make_shared<ChTriangleMeshShape>();
        rock3_mesh->SetMesh(rock_3_mmesh);
        rock3_mesh->SetBackfaceCull(true);
        rock3_Body->AddAsset(rock3_mesh);

        if (i == 0) {
            rock_5 = rock3_Body;
        } else {
            rock_6 = rock3_Body;
        }
    }

    //
    // THE DEFORMABLE TERRAIN
    //

    // Create the 'deformable terrain' object
    vehicle::SCMDeformableTerrain terrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    // Note: Irrlicht uses a Y-up frame
    terrain.SetPlane(ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(-CH_C_PI_2)));

    // Use a regular grid:
    double length = 14;
    double width = 4;
    terrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        terrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        terrain.SetSoilParameters(0.2e6,  // Bekker Kphi
                                  0,      // Bekker Kc
                                  1.1,    // Bekker n exponent
                                  0,      // Mohr cohesive limit (Pa)
                                  30,     // Mohr friction limit (degrees)
                                  0.01,   // Janosi shear coefficient (m)
                                  4e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                                  3e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
        );
    }

    // Set up bulldozing factors
    if (enable_bulldozing) {
        terrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        terrain.SetBulldozingParameters(
            55,  // angle of friction for erosion of displaced material at the border of the rut
            1,   // displaced material vs downward pressed material.
            5,   // number of erosion refinements per timestep
            6);  // number of concentric vertex selections subject to erosion
    }

    // We need to add a moving patch under every wheel
    // Or we can define a large moving patch at the pos of the rover body
    if (enable_moving_patch) {
        // add moving patch for the SCM terrain
        // the bodies were retrieved from the rover instance
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_LF)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_RF)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_LM)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_RM)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_LB)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rover.GetWheel(CuriosityWheelID::C_RB)->GetBody(), ChVector<>(0, 0, 0),
                               ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));

        // add moving patch for all obstacles
        terrain.AddMovingPatch(rock_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_5, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        terrain.AddMovingPatch(rock_6, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    terrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 20000);

    terrain.GetMesh()->SetWireframe(true);

    // Complete visual asset construction
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    while (application.GetDevice()->run()) {
        if (output) {
            // write drive torques of all six wheels into file
            csv << sys.GetChTime() << rover.GetWheelTracTorque(CuriosityWheelID::C_LF)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RF) << rover.GetWheelTracTorque(CuriosityWheelID::C_LM)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RM) << rover.GetWheelTracTorque(CuriosityWheelID::C_LB)
                << rover.GetWheelTracTorque(CuriosityWheelID::C_RB) << std::endl;
        }
        rover.Update();
        application.BeginScene();

        application.GetSceneManager()->getActiveCamera()->setTarget(
            core::vector3dfCH(rover.GetChassis()->GetBody()->GetPos()));
        application.DrawAll();

        application.DoStep();
        tools::drawColorbar(0, 20000, "Pressure yield [Pa]", application.GetDevice(), 1600);
        application.EndScene();

        ////std::cout << "--------- " << sys.GetChTime() << std::endl;
        ////terrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        // write output data into file
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
