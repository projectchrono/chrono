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
//   Demo code (advanced), about
//
//     - using the SCM semi-empirical model for deformable soil
//     - using a deformable tire
// =============================================================================

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_pardisomkl/ChSolverPardisoMKL.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/wheel/Wheel.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ReissnerTire.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Global parameter for tire:
    double tire_rad = 0.5;
    ChVector3d tire_center(0, tire_rad, 0);

    // Create a Chrono physical system
    ChSystemSMC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    std::shared_ptr<ChBody> mtruss(new ChBody);
    mtruss->SetFixed(true);
    sys.Add(mtruss);

    // CREATE A DEFORMABLE TIRE

    // The spindle
    auto spindle = chrono_types::make_shared<ChSpindle>();
    sys.Add(spindle);
    spindle->SetMass(80);
    spindle->SetInertiaXX(ChVector3d(1, 1, 1));
    spindle->SetPos(tire_center + ChVector3d(0, 0.2, 0));
    spindle->SetRot(QuatFromAngleZ(CH_PI_2));

    // The wheel object:
    auto wheel = chrono_types::make_shared<Wheel>(vehicle::GetDataFile("hmmwv/wheel/HMMWV_Wheel.json"));
    wheel->Initialize(nullptr, spindle, LEFT);

    // The tire:
    auto tire_reissner =
        chrono_types::make_shared<ReissnerTire>(vehicle::GetDataFile("hmmwv/tire/HMMWV_ReissnerTire.json"));
    tire_reissner->EnablePressure(false);
    tire_reissner->EnableContact(true);
    tire_reissner->SetContactSurfaceType(ChTire::ContactSurfaceType::TRIANGLE_MESH);
    tire_reissner->EnableRimConnection(true);
    std::static_pointer_cast<ChTire>(tire_reissner)->Initialize(wheel);
    tire_reissner->SetVisualizationType(VisualizationType::MESH);

    // Attach tire to wheel
    wheel->GetTire() = tire_reissner;

    // The motor that rotates the rim:
    auto motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    motor->SetSpindleConstraint(ChLinkMotorRotation::SpindleConstraint::OLDHAM);
    motor->SetAngleFunction(chrono_types::make_shared<ChFunctionRamp>(0, CH_PI / 4.0));
    motor->Initialize(spindle, mtruss, ChFrame<>(tire_center, QuatFromAngleY(CH_PI_2)));
    sys.Add(motor);

    // THE DEFORMABLE TERRAIN

    // Create the 'deformable terrain' object
    vehicle::SCMTerrain mterrain(&sys);

    // Displace/rotate the terrain reference plane.
    // Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
    // a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
    mterrain.SetPlane(ChCoordsys<>(ChVector3d(0, 0.2, 0.3), QuatFromAngleX(-CH_PI_2)));

    // Initialize the geometry of the soil: use either a regular grid:
    mterrain.Initialize(1.5, 6, 0.075);

    // Set the soil terramechanical parameters:
    mterrain.SetSoilParameters(1.2e6,  // Bekker Kphi
                               0,      // Bekker Kc
                               1.1,    // Bekker n exponent
                               0,      // Mohr cohesive limit (Pa)
                               30,     // Mohr friction limit (degrees)
                               0.01,   // Janosi shear coefficient (m)
                               5e7,    // Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                               2e4     // Damping (Pa s/m), proportional to negative vertical speed (optional)
    );
    mterrain.EnableBulldozing(true);  // inflate soil at the border of the rut
    mterrain.SetBulldozingParameters(
        55,   // angle of friction for erosion of displaced material at the border of the rut
        0.8,  // displaced material vs downward pressed material.
        5,    // number of erosion refinements per timestep
        10);  // number of concentric vertex selections subject to erosion

    // Set some visualization parameters: either with a texture, or with falsecolor plot, etc.
    // mterrain.SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE, 0, 30000.2);
    mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_PRESSURE_YELD, 0, 30000.2);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE, 0, 0.15);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE_PLASTIC, 0, 0.15);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_SINKAGE_ELASTIC, 0, 0.05);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_STEP_PLASTIC_FLOW, 0, 0.0001);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_ISLAND_ID, 0, 8);
    // mterrain.SetPlotType(vehicle::SCMTerrain::PLOT_IS_TOUCHED, 0, 8);
    mterrain.GetMesh()->SetWireframe(true);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Deformable soil and deformable tire");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(1.0, 1.4, -1.2), ChVector3d(0, tire_rad, 0));
    vis->AddLightDirectional();
    vis->AddLightWithShadow(ChVector3d(1.5, 5.5, -2.5), ChVector3d(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                            ChColor(0.8f, 0.8f, 1.0f));
    vis->EnableShadows();

    // THE SOFT-REAL-TIME CYCLE

    // change the solver to PardisoMKL:
    std::cout << "Using PardisoMKL solver\n";
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    mkl_solver->LockSparsityPattern(true);
    sys.SetSolver(mkl_solver);

    // Change the timestepper to HHT:
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    auto integrator = std::static_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper());
    integrator->SetAlpha(-0.2);
    integrator->SetMaxIters(8);
    integrator->SetAbsTolerances(1e-1, 10);
    integrator->SetModifiedNewton(false);
    integrator->SetVerbose(true);

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        tools::drawColorbar(vis.get(), 0, 30000, "Pressure yield [Pa]", 1180);
        vis->EndScene();
        sys.DoStepDynamics(0.002);
    }

    return 0;
}
