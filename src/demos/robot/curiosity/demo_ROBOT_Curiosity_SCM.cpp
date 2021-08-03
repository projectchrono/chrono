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

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChInertiaUtils.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/physics/ChParticlesClones.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkDistance.h"

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
// The options are Chassis_Type::Scarecrow and Chassis_Type::FullRover
Chassis_Type chassis_type = Chassis_Type::FullRover;

// Specify rover wheel type
// The options are Wheel_Type::RealWheel, Wheel_Type::SimpleWheel, and Wheel_Type::CylWheel
Wheel_Type wheel_type = Wheel_Type::RealWheel;

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

// Use custom material for the Viper Wheel
bool use_custom_mat = true;

// Return customized wheel material parameters
std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.65f;  // coefficient of friction
    float cr = 0.1f;   // coefficient of restitution
    float Y = 2e7f;    // Young's modulus
    float nu = 0.3f;   // Poisson ratio
    float kn = 2e5f;   // normal stiffness
    float gn = 40.0f;  // normal viscous damping
    float kt = 2e5f;   // tangential stiffness
    float gt = 20.0f;  // tangential viscous damping

    switch (contact_method) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            matNSC->SetFriction(mu);
            matNSC->SetRestitution(cr);
            return matNSC;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChMaterialSurfaceSMC>();
            matSMC->SetFriction(mu);
            matSMC->SetRestitution(cr);
            matSMC->SetYoungModulus(Y);
            matSMC->SetPoissonRatio(nu);
            matSMC->SetKn(kn);
            matSMC->SetGn(gn);
            matSMC->SetKt(kt);
            matSMC->SetGt(gt);
            return matSMC;
        }
        default:
            return std::shared_ptr<ChMaterialSurface>();
    }
}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Global parameter for moving patch size:
    double wheel_range = 0.5;

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Curiosity Obstacle Crossing on SCM", core::dimension2d<u32>(1800, 1000),
                         VerticalDir::Y, false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(2.0f, 1.4f, 0.0f), core::vector3df(0, (f32)wheel_range, 0));
    application.AddLightWithShadow(core::vector3df(-5.0f, 8.0f, -0.5f), core::vector3df(-1.0, 0, 0), 100, 1, 35, 85,
                                   512, video::SColorf(0.8f, 0.8f, 1.0f));

    // Initialize output
    if (output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
    }
    utils::CSV_writer csv(" ");

    // Viper rover initial position and orientation
    ChVector<double> body_pos(-5, -0.2, 0);
    ChQuaternion<> body_rot = Q_from_AngX(-CH_C_PI / 2);

    std::shared_ptr<CuriosityRover> rover;

    if (use_custom_mat == true) {
        // if customize wheel material
        rover = chrono_types::make_shared<CuriosityRover>(
            &my_system, body_pos, body_rot, CustomWheelMaterial(ChContactMethod::SMC), chassis_type, wheel_type);

        // the user can choose to enable DC motor option
        // if the DC motor option has been enabled, the rotational speed will be switched to no-load-speed of the DC
        // motor Note: This function has to be called before initialization
        // motor defaut linear relationship is set to stall torque 500 N-m, and no load speed 3.1415 rad/s
        rover->SetDCControl(true);

        // The user can also choose to use simplified wheel
        /*
        rover = chrono_types::make_shared<CuriosityRover>(&my_system,
                                                            body_pos,
                                                            body_rot,
                                                            CustomWheelMaterial(ChContactMethod::SMC),
                                                            Chassis_Type::FullRover,
                                                            Wheel_Type::SimpleWheel);
        */

        rover->Initialize();

        // Default value is w = 3.1415 rad/s
        // User can define using SetMotorSpeed
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::LF);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::RF);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::LM);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::RM);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::LB);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::RB);

    } else {
        // if use default material
        rover = chrono_types::make_shared<CuriosityRover>(&my_system, body_pos, body_rot, chassis_type, wheel_type);

        // The user can also choose to use simplified wheel
        /*
        rover = chrono_types::make_shared<CuriosityRover>(&my_system,
                                                        body_pos,
                                                        body_rot,
                                                        Chassis_Type::FullRover,
                                                        Wheel_Type::SimpleWheel);
        */

        // the user can choose to enable DC motor option
        // if the DC motor option has been enabled, the rotational speed will be switched to no-load-speed of the DC
        // motor Note: This function has to be called before initialization
        // motor defaut linear relationship is set to stall torque 500 N-m, and no load speed 3.1415 rad/s
        rover->SetDCControl(true);
        rover->Initialize();

        // Default value is w = 3.1415 rad/s
        // User can define using SetMotorSpeed
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::LF);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::RF);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::LM);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::RM);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::LB);
        // curiosity->SetMotorSpeed(CH_C_PI,WheelID::RB);
    }

    std::shared_ptr<ChBodyAuxRef> rock_1;
    std::shared_ptr<ChBodyAuxRef> rock_2;
    std::shared_ptr<ChBodyAuxRef> rock_3;
    std::shared_ptr<ChBodyAuxRef> rock_4;
    std::shared_ptr<ChBodyAuxRef> rock_5;
    std::shared_ptr<ChBodyAuxRef> rock_6;

    // create default SMC materials for the obstacles
    std::shared_ptr<ChMaterialSurface> rockSufaceMaterial = CustomWheelMaterial(ChContactMethod::SMC);

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
        my_system.Add(rock1_Body);

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
        my_system.Add(rock2_Body);

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
        my_system.Add(rock3_Body);

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
    vehicle::SCMDeformableTerrain mterrain(&my_system);

    // Displace/rotate the terrain reference plane.
    // Note that SCMDeformableTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    // Note: Irrlicht uses a Y-up frame
    mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, -0.5, 0), Q_from_AngX(-CH_C_PI_2)));

    // Use a regular grid:
    double length = 14;
    double width = 4;
    mterrain.Initialize(length, width, mesh_resolution);

    // Set the soil terramechanical parameters
    if (var_params) {
        // Here we use the soil callback defined at the beginning of the code
        auto my_params = chrono_types::make_shared<MySoilParams>();
        mterrain.RegisterSoilParametersCallback(my_params);
    } else {
        // If var_params is set to be false, these parameters will be used
        mterrain.SetSoilParameters(0.2e6,  // Bekker Kphi
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
        mterrain.EnableBulldozing(true);  // inflate soil at the border of the rut
        mterrain.SetBulldozingParameters(
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
        mterrain.AddMovingPatch(rover->GetWheelBody(WheelID::LF), ChVector<>(0, 0, 0),
                                ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rover->GetWheelBody(WheelID::RF), ChVector<>(0, 0, 0),
                                ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rover->GetWheelBody(WheelID::LM), ChVector<>(0, 0, 0),
                                ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rover->GetWheelBody(WheelID::RM), ChVector<>(0, 0, 0),
                                ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rover->GetWheelBody(WheelID::LB), ChVector<>(0, 0, 0),
                                ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rover->GetWheelBody(WheelID::RB), ChVector<>(0, 0, 0),
                                ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));

        // add moving patch for all obstacles
        mterrain.AddMovingPatch(rock_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rock_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rock_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rock_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rock_5, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(rock_6, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 20000);

    mterrain.GetMesh()->SetWireframe(true);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    application.SetTimestep(0.001);

    while (application.GetDevice()->run()) {
        if (output) {
            // this example writeout will write drive torques of all six wheels into file
            csv << my_system.GetChTime() << rover->GetWheelTracTorque(WheelID::LF)
                << rover->GetWheelTracTorque(WheelID::RF) << rover->GetWheelTracTorque(WheelID::LM)
                << rover->GetWheelTracTorque(WheelID::RM) << rover->GetWheelTracTorque(WheelID::LB)
                << rover->GetWheelTracTorque(WheelID::RB) << std::endl;
        }
        rover->Update();
        application.BeginScene();

        application.GetSceneManager()->getActiveCamera()->setTarget(
            core::vector3dfCH(rover->GetChassisBody()->GetPos()));
        application.DrawAll();

        application.DoStep();
        tools::drawColorbar(0, 20000, "Pressure yield [Pa]", application.GetDevice(), 1600);
        application.EndScene();

        ////std::cout << "--------- " << my_system.GetChTime() << std::endl;
        ////mterrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        // write output data into file
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
