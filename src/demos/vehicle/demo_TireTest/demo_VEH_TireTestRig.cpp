// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2015 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// =============================================================================

#include <algorithm>

#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_LugreTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;

double step_size = 1e-3;
double tire_step_size = 1e-4;

int main() {
    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = std::make_shared<hmmwv::HMMWV_WheelLeft>("Wheel");

    ////auto tire = std::make_shared<hmmwv::HMMWV_RigidTire>("Rigid tire");
    ////auto tire = std::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");
    auto tire = std::make_shared<hmmwv::HMMWV_FialaTire>("Fiala tire");
    ////auto tire = std::make_shared<hmmwv::HMMWV_LugreTire>("Lugre tire");

    // Create and initialize test rig
    // ------------------------------

    ChTireTestRig rig(wheel, tire);

    ////rig.SetGravitationalAcceleration(0);
    rig.SetNormalLoad(8000);

    ////rig.SetCamberAngle(+15 * CH_C_DEG_TO_RAD);

    ////rig.SetLongSpeedFunction(std::make_shared<ChFunction_Const>(1.0));
    ////rig.SetAngSpeedFunction(std::make_shared<ChFunction_Const>(1.0));
    ////rig.SetSlipAngleFunction(std::make_shared<ChFunction_Sine>(0, 0.6, 0.2));

    rig.SetTireStepsize(tire_step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    rig.SetTerrainRigid(0.8, 2e7, 0);
    ////rig.SetTerrainSCM(6259.1e3, 5085.6e3, 1.42, 1.58e3, 34.1, 22.17e-3);

    ////rig.Initialize();
    rig.Initialize(0.2, 1.0);

    // Create the Irrlicht visualization
    // ---------------------------------

    ChIrrApp application(&rig.GetSystem(), L"Tire Test Rig", irr::core::dimension2d<irr::u32>(1920, 1080), false, true);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(irr::core::vector3df(1, 1, 1), irr::core::vector3df(0, 0, 0));

    application.AssetBindAll();
    application.AssetUpdateAll();

    auto camera = application.GetSceneManager()->getActiveCamera();
    camera->setFOV(irr::core::PI / 4.5f);

    // Perform the simulation
    // ----------------------

    rig.GetSystem().SetupInitial();

    while (application.GetDevice()->run()) {
        auto& loc = rig.GetPos();
        auto x = (irr::f32)loc.x();
        auto y = (irr::f32)loc.y();
        auto z = (irr::f32)loc.z();
        camera->setPosition(irr::core::vector3df(x + 1, y + 1.5f, z + 2));
        camera->setTarget(irr::core::vector3df(x, y, z));

        application.BeginScene();
        application.DrawAll();
        rig.Advance(step_size);
        application.EndScene();

        ////std::cout << rig.GetSystem().GetChTime() << std::endl;
        ////auto long_slip = tire->GetLongitudinalSlip();
        ////auto slip_angle = tire->GetSlipAngle();
        ////auto camber_angle = tire->GetCamberAngle();
        ////std::cout << long_slip << " " << slip_angle << " " << camber_angle << std::endl;
        ////auto tforce = rig.GetTireForce();
        ////auto frc = tforce.force;
        ////auto pnt = tforce.point;
        ////auto trq = tforce.moment;
        ////std::cout << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
        ////std::cout << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
        ////std::cout << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
    }

    return 0;
}
