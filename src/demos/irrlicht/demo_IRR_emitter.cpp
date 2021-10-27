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
// Authors: Alessandro Tasora
// =============================================================================
//
// Demo code about
// - using the ChParticleEmitter to create flows of random shapes
// - use Irrlicht to display objects.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/particlefactory/ChParticleRemover.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the main namespace of Chrono, and other chrono namespaces
using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a ChronoENGINE physical system
    ChSystemNSC mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Particle emitter", core::dimension2d<u32>(800, 600));
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(5, 7, -10));

    //
    // CREATE THE SYSTEM OBJECTS
    //

    // Create the floor:
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto floorBody = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true, floor_mat);
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);

    auto mtexture = chrono_types::make_shared<ChTexture>();
    mtexture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    floorBody->AddAsset(mtexture);

    mphysicalSystem.Add(floorBody);

    // Create an emitter:

    ChParticleEmitter emitter;

    // Ok, that object will take care of generating particle flows for you.
    // It accepts a lot of settings, for creating many different types of particle
    // flows, like fountains, outlets of various shapes etc.

    // Set the flow rate [particles/s]:
    emitter.ParticlesPerSecond() = 20;

    // Alternative: flow defined by mass, [kg/s]:
    emitter.SetFlowControlMode(ChParticleEmitter::FLOW_MASSPERSECOND);
    emitter.MassPerSecond() = 1000;

    // Optional: limit the total n. of particles that can be generated
    emitter.SetUseParticleReservoir(true);
    emitter.ParticleReservoirAmount() = 200;

    // Optional: limit the total mass that can be generated
    emitter.SetUseMassReservoir(true);
    emitter.MassReservoirAmount() = 5000;

    // Our ChParticleEmitter object, among the main settings, it requires
    // that you give him four 'randomizer' objects: one is in charge of
    // generating random shapes, one is in charge of generating
    // random positions, one for random alignments, and one for random velocities.
    // In the following we need to instance such objects. (There are many ready-to-use
    // randomizer objects already available in chrono, but note that you could also
    // inherit your own class from these randomizers if the choice is not enough).

    // ---Initialize the randomizer for positions
    auto emitter_positions = chrono_types::make_shared<ChRandomParticlePositionRectangleOutlet>();
    emitter_positions->Outlet() =
        ChCoordsys<>(ChVector<>(0, 3, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X));  // center and alignment of the outlet
    emitter_positions->OutletWidth() = 3.0;
    emitter_positions->OutletHeight() = 4.5;

    emitter.SetParticlePositioner(emitter_positions);

    // ---Initialize the randomizer for alignments
    auto emitter_rotations = chrono_types::make_shared<ChRandomParticleAlignmentUniform>();

    emitter.SetParticleAligner(emitter_rotations);

    // ---Initialize the randomizer for velocities, with statistical distribution
    auto mvelo = chrono_types::make_shared<ChRandomParticleVelocityConstantDirection>();
    mvelo->SetDirection(-VECT_Y);
    mvelo->SetModulusDistribution(0.0);

    emitter.SetParticleVelocity(mvelo);

    // ---Initialize the randomizer for creations, with statistical distribution

    // Create a ChRandomShapeCreator object (ex. here for box particles)
    auto mcreator_boxes = chrono_types::make_shared<ChRandomShapeCreatorBoxes>();
    mcreator_boxes->SetXsizeDistribution(
        chrono_types::make_shared<ChZhangDistribution>(0.5, 0.2));  // Zhang parameters: average val, min val.
    mcreator_boxes->SetSizeRatioZDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.2, 1.0));
    mcreator_boxes->SetSizeRatioYZDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.4, 1.0));
    mcreator_boxes->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1000));

    // Optional: define a callback to be exectuted at each creation of a box particle:
    class MyCreator_boxes : public ChRandomShapeCreator::AddBodyCallback {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> mbody,
                               ChCoordsys<> mcoords,
                               ChRandomShapeCreator& mcreator) override {
            // Ex.: attach some optional assets, ex for visualization
            auto mvisual = chrono_types::make_shared<ChColorAsset>();
            mvisual->SetColor(ChColor(0.0f, 1.0f, (float)ChRandom()));
            mbody->AddAsset(mvisual);
        }
    };
    auto callback_boxes = chrono_types::make_shared<MyCreator_boxes>();
    mcreator_boxes->RegisterAddBodyCallback(callback_boxes);

    // Finally, tell to the emitter that it must use the 'mixer' above:
    emitter.SetParticleCreator(mcreator_boxes);

    // --- Optional: what to do by default on ALL newly created particles?
    //     A callback executed at each particle creation can be attached to the emitter.
    //     For example, we need that new particles will be bound to Irrlicht visualization:

    // a- define a class that implement your custom OnAddBody method...
    class MyCreatorForAll : public ChRandomShapeCreator::AddBodyCallback {
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> mbody,
                               ChCoordsys<> mcoords,
                               ChRandomShapeCreator& mcreator) override {
            // Enable Irrlicht visualization for all particles
            airrlicht_application->AssetBind(mbody);
            airrlicht_application->AssetUpdate(mbody);

            // Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
            mbody->SetNoGyroTorque(true);
        }
        ChIrrApp* airrlicht_application;
    };
    // b- create the callback object...
    auto mcreation_callback = chrono_types::make_shared<MyCreatorForAll>();
    // c- set callback own data that he might need...
    mcreation_callback->airrlicht_application = &application;
    // d- attach the callback to the emitter!
    emitter.RegisterAddBodyCallback(mcreation_callback);

    // Use this function for adding a ChIrrNodeAsset to all already created items (ex. the floor, etc.)
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetSolverType(ChSolver::Type::PSOR);
    mphysicalSystem.SetSolverMaxIterations(40);

    application.SetTimestep(0.02);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        // Continuosly create particle flow:
        emitter.EmitParticles(mphysicalSystem, application.GetTimestep());

        application.DoStep();

        application.EndScene();
    }

    return 0;
}
