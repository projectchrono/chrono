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

#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_models/vehicle/hmmwv/HMMWV_ANCFTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_FialaTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_LugreTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_ReissnerTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_RigidTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_TMeasyTire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac89Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Pac02Tire.h"
#include "chrono_models/vehicle/hmmwv/HMMWV_Wheel.h"

#include "chrono_vehicle/utils/ChUtilsJSON.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChTireTestRig.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::irrlicht;

double step_size = 1e-3;
double tire_step_size = 1e-4;

int main() {
    // Create system
    // -------------

    ChSystemNSC system;

    system.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    system.SetSolverMaxIterations(150);
    system.SetMaxPenetrationRecoverySpeed(4.0);

    // Create wheel and tire subsystems
    // --------------------------------

    auto wheel = chrono_types::make_shared<hmmwv::HMMWV_Wheel>("Wheel");

    ////auto tire = chrono_types::make_shared<hmmwv::HMMWV_RigidTire>("Rigid tire");
    ////auto tire = chrono_types::make_shared<hmmwv::HMMWV_TMeasyTire>("TMeasy tire");
    ////auto tire = chrono_types::make_shared<hmmwv::HMMWV_FialaTire>("Fiala tire");
    ////auto tire = chrono_types::make_shared<hmmwv::HMMWV_Pac89Tire>("Pac89 tire");
    ////auto tire = chrono_types::make_shared<hmmwv::HMMWV_Pac02Tire>("Pac02 tire");
    ////auto tire = chrono_types::make_shared<hmmwv::HMMWV_LugreTire>("Lugre tire");

    ////std::string tire_file("hmmwv/tire/HMMWV_RigidTire.json");
    ////std::string tire_file("hmmwv/tire/HMMWV_FialaTire.json");
    ////std::string tire_file("hmmwv/tire/HMMWV_TMeasyTire.json");
    ////std::string tire_file("hmmwv/tire/HMMWV_Pac89Tire.json");
    std::string tire_file("hmmwv/tire/HMMWV_Pac02Tire.json");
    auto tire = ReadTireJSON(vehicle::GetDataFile(tire_file));

    // Create and configure test rig
    // -----------------------------

    ChTireTestRig rig(wheel, tire, &system);

    ////rig.SetGravitationalAcceleration(0);
    rig.SetNormalLoad(8000);

    ////rig.SetCamberAngle(+15 * CH_C_DEG_TO_RAD);

    rig.SetTireStepsize(tire_step_size);
    rig.SetTireCollisionType(ChTire::CollisionType::FOUR_POINTS);
    rig.SetTireVisualizationType(VisualizationType::MESH);

    rig.SetTerrainRigid(0.8, 0, 2e7);
    ////rig.SetTerrainSCM(6259.1e3, 5085.6e3, 1.42, 1.58e3, 34.1, 22.17e-3);

    // Set test scenario
    // -----------------

    // Scenario: driven wheel
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10.0));
    ////rig.Initialize();

    // Scenario: pulled wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(1.0));
    ////rig.Initialize();

    // Scenario: imobilized wheel
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.0));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.0));
    ////rig.Initialize();

    // Scenario: prescribe all motion functions
    ////rig.SetLongSpeedFunction(chrono_types::make_shared<ChFunction_Const>(0.2));
    ////rig.SetAngSpeedFunction(chrono_types::make_shared<ChFunction_Const>(10.0));
    ////rig.SetSlipAngleFunction(chrono_types::make_shared<ChFunction_Sine>(0, 0.6, 0.2));
    ////rig.Initialize();

    // Scenario: specified longitudinal slip
    rig.Initialize(0.2, 1.0);

    // Create the Irrlicht visualization
    // ---------------------------------

    ChIrrApp application(&system, L"Tire Test Rig", irr::core::dimension2d<irr::u32>(1280, 720), VerticalDir::Z);
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera();

    application.AssetBindAll();
    application.AssetUpdateAll();

    auto camera = application.GetActiveCamera();
    camera->setFOV(irr::core::PI / 4.5f);

    // Perform the simulation
    // ----------------------

    while (application.GetDevice()->run()) {
        auto& loc = rig.GetPos();
        auto x = (irr::f32)loc.x();
        auto y = (irr::f32)loc.y();
        auto z = (irr::f32)loc.z();
        camera->setPosition(irr::core::vector3df(x + 1.0f, y + 2.5f, z + 1.5f));
        camera->setTarget(irr::core::vector3df(x, y + 0.25f, z));

        application.BeginScene();
        application.DrawAll();
        rig.Advance(step_size);
        application.EndScene();

        ////std::cout << system.GetChTime() << std::endl;
        ////auto long_slip = tire->GetLongitudinalSlip();
        ////auto slip_angle = tire->GetSlipAngle();
        ////auto camber_angle = tire->GetCamberAngle();
        ////std::cout << "   " << long_slip << " " << slip_angle << " " << camber_angle << std::endl;
        ////auto tforce = rig.ReportTireForce();
        ////auto frc = tforce.force;
        ////auto pnt = tforce.point;
        ////auto trq = tforce.moment;
        ////std::cout << "   " << frc.x() << " " << frc.y() << " " << frc.z() << std::endl;
        ////std::cout << "   " << pnt.x() << " " << pnt.y() << " " << pnt.z() << std::endl;
        ////std::cout << "   " << trq.x() << " " << trq.y() << " " << trq.z() << std::endl;
    }

    return 0;
}
