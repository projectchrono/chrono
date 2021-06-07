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
// Demo to show Viper Rover operated on SCM Terrain
//
// =============================================================================

#include "chrono_models/robot/viper/Viper.h"

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

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::geometry;
using namespace chrono::viper;

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

// Custom callback for setting location-dependent soil properties.
// Note that the location is given in the SCM reference frame.
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
bool use_custom_mat = false;

// Return customized wheel material parameters
std::shared_ptr<ChMaterialSurface> CustomWheelMaterial(ChContactMethod contact_method) {
    float mu = 0.4f;   // coefficient of friction
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
    ////double body_range = 1.2;

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;
    my_system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Viper Rover on SCM", core::dimension2d<u32>(1280, 720), VerticalDir::Z, false, true);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, 30.f, 100.f), irr::core::vector3df(30.f, -30.f, 100.f));
    application.AddTypicalCamera(core::vector3df(2.0f, 0.0f, 1.4f), core::vector3df(0, 0, (f32)wheel_range));
    application.AddLightWithShadow(core::vector3df(1.5f, -2.5f, 5.5f), core::vector3df(0, 0, 0), 10, 2.2, 15, 0, 512,
                                   video::SColorf(0.8f, 0.8f, 1.0f));

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

    // Create a Viper Rover instance
    std::shared_ptr<ChBodyAuxRef> Wheel_1;
    std::shared_ptr<ChBodyAuxRef> Wheel_2;
    std::shared_ptr<ChBodyAuxRef> Wheel_3;
    std::shared_ptr<ChBodyAuxRef> Wheel_4;
    std::shared_ptr<ChBodyAuxRef> Body_1;

    if (use_custom_mat == true) {
        // if customize wheel material
        ViperRover viper(&my_system, body_pos, QUNIT, CustomWheelMaterial(ChContactMethod::SMC));
        viper.Initialize();

        // Default value is w = 3.1415 rad/s
        // User can define using SetMotorSpeed
        // viper.SetMotorSpeed(CH_C_PI,WheelID::LF);
        // viper.SetMotorSpeed(CH_C_PI,WheelID::RF);
        // viper.SetMotorSpeed(CH_C_PI,WheelID::LB);
        // viper.SetMotorSpeed(CH_C_PI,WheelID::RB);

        // Get wheels and bodies to set up SCM patches
        Wheel_1 = viper.GetWheelBody(WheelID::LF);
        Wheel_2 = viper.GetWheelBody(WheelID::RF);
        Wheel_3 = viper.GetWheelBody(WheelID::LB);
        Wheel_4 = viper.GetWheelBody(WheelID::RB);
        Body_1 = viper.GetChassisBody();
    } else {
        // if use default material
        ViperRover viper(&my_system, body_pos, QUNIT);
        viper.Initialize();

        // viper.SetMotorSpeed(CH_C_PI * 2,WheelID::LF);
        // viper.SetMotorSpeed(CH_C_PI * 2,WheelID::RF);
        // viper.SetMotorSpeed(CH_C_PI * 2,WheelID::LB);
        // viper.SetMotorSpeed(CH_C_PI * 2,WheelID::RB);

        // Get wheels and bodies to set up SCM patches
        Wheel_1 = viper.GetWheelBody(WheelID::LF);
        Wheel_2 = viper.GetWheelBody(WheelID::RF);
        Wheel_3 = viper.GetWheelBody(WheelID::LB);
        Wheel_4 = viper.GetWheelBody(WheelID::RB);
        Body_1 = viper.GetChassisBody();
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
    mterrain.SetPlane(ChCoordsys<>(ChVector<>(0, 0, -0.5)));

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
        mterrain.AddMovingPatch(Wheel_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(Wheel_2, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(Wheel_3, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        mterrain.AddMovingPatch(Wheel_4, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * wheel_range, 2 * wheel_range));
        /// mterrain.AddMovingPatch(Body_1, ChVector<>(0, 0, 0), ChVector<>(0.5, 2 * body_range, 2 * body_range))
    }

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    mterrain.SetPlotType(vehicle::SCMDeformableTerrain::PLOT_PRESSURE, 0, 20000);

    mterrain.SetMeshWireframe(true);

    // ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
    application.AssetBindAll();

    // ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
    application.AssetUpdateAll();

    // Use shadows in realtime view
    application.AddShadowAll();

    application.SetTimestep(0.0005);

    while (application.GetDevice()->run()) {
        if (output) {
            // vehicle::TerrainForce frc = mterrain.GetContactForce(mrigidbody);
            // csv << my_system.GetChTime() << frc.force << frc.moment << frc.point << std::endl;
        }
        application.BeginScene();

        application.GetActiveCamera()->setTarget(core::vector3dfCH(Body_1->GetPos()));
        application.DrawAll();

        application.DoStep();
        tools::drawColorbar(0, 20000, "Pressure yield [Pa]", application.GetDevice(), 1180);
        application.EndScene();

        ////mterrain.PrintStepStatistics(std::cout);
    }

    if (output) {
        csv.write_to_file(out_dir + "/output.dat");
    }

    return 0;
}
