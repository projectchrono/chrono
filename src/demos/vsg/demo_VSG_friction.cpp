#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/assets/ChTexture.h"
#include "chrono_vsg/ChVSGApp.h"

using namespace chrono;
using namespace geometry;
using namespace chrono::vsg3d;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2020 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a physical system
    ChSystemNSC mphysicalSystem;

    mphysicalSystem.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Create all the rigid bodies.
    double mradius = 0.5;
    double density = 1000;

    // Create a texture asset. It can be shared between bodies.
    auto textureasset = chrono_types::make_shared<ChTexture>(GetChronoDataFile("vsg/textures/Marble010.jpg"));

    // Create some spheres that roll horizontally, with increasing rolling friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        mat->SetFriction(0.4f);
        mat->SetRollingFriction(((float)bi / 10) * 0.05f);

        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,  // radius size
                                                                       density,  // density
                                                                       true,     // visualization?
                                                                       true,     // collision?
                                                                       mat);     // contact material
        msphereBody->AddAsset(textureasset);
        // Set some properties
        msphereBody->SetPos(ChVector<>(-7, mradius - 0.5, -5 + bi * mradius * 2.5));
        // msphereBody->AddAsset(textureasset);  // assets can be shared

        // Set initial speed: rolling in horizontal direction
        double initial_angspeed = 10;
        double initial_linspeed = initial_angspeed * mradius;
        msphereBody->SetWvel_par(ChVector<>(0, 0, -initial_angspeed));
        msphereBody->SetPos_dt(ChVector<>(initial_linspeed, 0, 0));

        // Add to the system
        mphysicalSystem.Add(msphereBody);
    }

    // Create some spheres that spin on place, for a 'drilling friction' case, with increasing spinning friction values
    for (int bi = 0; bi < 10; bi++) {
        auto mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
        mat->SetFriction(0.4f);
        mat->SetSpinningFriction(((float)bi / 10) * 0.02f);

        auto msphereBody = chrono_types::make_shared<ChBodyEasySphere>(mradius,  // radius size
                                                                       density,  // density
                                                                       true,     // visualization?
                                                                       true,     // collision?
                                                                       mat);     // contact material
        msphereBody->AddAsset(textureasset);
        // Set some properties
        msphereBody->SetPos(ChVector<>(-8, 1 + mradius - 0.5, -5 + bi * mradius * 2.5));
        // msphereBody->AddAsset(textureasset);  // assets can be shared

        // Set initial speed: spinning in vertical direction
        msphereBody->SetWvel_par(ChVector<>(0, 20, 0));

        // Add to the system
        mphysicalSystem.Add(msphereBody);

        // Notes:
        // - setting nonzero spinning friction and/or setting nonzero rolling friction
        //   affects the speed of the solver (each contact eats 2x of CPU time repsect to the
        //   case of simple sliding/staic contact)
        // - avoid using zero spinning friction with nonzero rolling friction.
    }

    // Create a container fixed to ground
    auto bin = chrono_types::make_shared<ChBody>();
    bin->SetPos(ChVector<>(0, -1, 0));
    bin->SetBodyFixed(true);
    bin->SetCollide(true);

    // Set rolling and friction coefficients for the container.
    // By default, the composite material will use the minimum value for an interacting collision pair.
    auto bin_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    bin_mat->SetRollingFriction(1);
    bin_mat->SetSpinningFriction(1);

    // Add collision geometry and visualization shapes for the floor and the 4 walls
    bin->GetCollisionModel()->ClearModel();
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20, 1, 20) / 2.0, ChVector<>(0, 0, 0));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(1, 2, 20.99) / 2.0, ChVector<>(-10, 1, 0));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(1, 2, 20.99) / 2.0, ChVector<>(10, 1, 0));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20.99, 2, 1) / 2.0, ChVector<>(0, 1, -10));
    utils::AddBoxGeometry(bin.get(), bin_mat, ChVector<>(20.99, 2, 1) / 2.0, ChVector<>(0, 1, 10));
    bin->GetCollisionModel()->BuildModel();

    bin->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("vsg/textures/Wood034.jpg")));

    mphysicalSystem.Add(bin);

    ChVSGApp app;

    // Modify some setting of the physical system for the simulation
    mphysicalSystem.SetSolverType(ChSolver::Type::APGD);
    mphysicalSystem.SetSolverMaxIterations(100);

    app.setTimeStep(0.005);
    app.setOutputStep(0.015);
    app.setUpVector(ChVector<>(0.0, 0.0, -1.0));

    bool ok = app.Initialize(800, 1000, "VSG Friction Demo", &mphysicalSystem);
    double modelTime = 0.0;
    double maxModelTime = 10.0;
    ChTimer timer;
    timer.reset();  // mandatory on macos, doesn't do no harm on windows and linux
    timer.start();
    while (app.GetViewer()->advanceToNextFrame()) {
        modelTime = mphysicalSystem.GetChTime();
        if (modelTime >= maxModelTime) {
            GetLog() << "Max. model time of " << maxModelTime << " s reached. Terminate.\n";
            timer.stop();
            GetLog() << "Max. wallclock time is " << timer.GetTimeSeconds() << " s.\n";
            app.GetViewer()->close();
        }
        app.doTimeStep();
        app.Render();
    }
    return 0;
}
