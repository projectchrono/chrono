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
// Demo code about using the ChParticleEmitter to create flows of random shapes
// from different distributions
//
// =============================================================================

#include "chrono/assets/ChTexture.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/particlefactory/ChParticleRemover.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_postprocess/ChPovRay.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the main namespace of Chrono, and other chrono namespaces
using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono system and set the associated collision system
    ChSystemNSC sys;
    sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Particle emitter: creation from various distributions");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 4, -6), ChVector<>(0, -2, 0));

    // Create an exporter to POVray !!
    ChPovRay pov_exporter = ChPovRay(&sys);

    // Important: set the path to the template:
    pov_exporter.SetTemplateFile(GetChronoDataFile("POVRay_chrono_template.pov"));

    // Set the path where it will save all .pov, .ini, .asset and .dat files, a directory will be created if not
    // existing
    pov_exporter.SetBasePath(GetChronoOutputPath() + "EMIT_CREATION");

    pov_exporter.SetLight(VNULL, ChColor(0, 0, 0), false);
    pov_exporter.SetCustomPOVcommandsScript(
        " \
         camera { \
              angle    45 \
              location <3.0 , 2.5 ,-18.0> \
              right    x*image_width/image_height \
              look_at  <0 , -2, 0> \
              rotate   <0,-180*(clock),0> \
          } \
	     light_source {   \
              <6, 15, -6>  \
	          color rgb<1.2,1.2,1.2> \
              area_light <5, 0, 0>, <0, 0, 5>, 8, 8 \
              adaptive 1 \
              jitter\
            } \
         box \
            {  \
                <20, 16, 20>, <0, 16, 0> \
                texture{ pigment{color rgb<3,3,3> }}    \
                finish { ambient 1 } \
            } \
          ");

    // CREATE THE SYSTEM OBJECTS

    // Create the floor:
    auto floor_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();

    auto floor_body = chrono_types::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true, floor_mat);
    floor_body->SetPos(ChVector<>(0, -5, 0));
    floor_body->SetBodyFixed(true);
    floor_body->GetVisualShape(0)->SetColor(ChColor(0.0f, 1.0f, (float)ChRandom()));

    auto shape1 = chrono_types::make_shared<ChCollisionShapeBox>(floor_mat, 20, 1, 20);
    auto shape2 = chrono_types::make_shared<ChCollisionShapeBox>(floor_mat, 2, 24, 40);
    auto shape3 = chrono_types::make_shared<ChCollisionShapeBox>(floor_mat, 20, 24, 2);

    floor_body->AddCollisionShape(shape1);
    floor_body->AddCollisionShape(shape2, ChFrame<>(ChVector<>(-5, 0, 0), QUNIT));
    floor_body->AddCollisionShape(shape2, ChFrame<>(ChVector<>(5, 0, 0), QUNIT));
    floor_body->AddCollisionShape(shape3, ChFrame<>(ChVector<>(0, 0, -5), QUNIT));
    floor_body->AddCollisionShape(shape3, ChFrame<>(ChVector<>(0, 0, 5), QUNIT));

    // Custom rendering in POVray:
    pov_exporter.SetCustomCommands(floor_body,
                                   "texture{ pigment{ color rgb<1,1,1>}} \n\
                             texture{ Raster(4, 0.02, rgb<0.8,0.8,0.8>) } \n\
                             texture{ Raster(4, 0.02, rgb<0.8,0.8,0.8>) rotate<0,90,0> } \n\
                             texture{ Raster(4*0.2, 0.04, rgb<0.8,0.8,0.8>) } \n\
                             texture{ Raster(4*0.2, 0.04, rgb<0.8,0.8,0.8>) rotate<0,90,0> } \n\
                              ");

    sys.Add(floor_body);

    // Create an emitter:
    ChParticleEmitter emitter;

    // Ok, that object will take care of generating particle flows for you.
    // It accepts a lot of settings, for creating many different types of particle
    // flows, like fountains, outlets of various shapes etc.
    // For instance, set the flow rate, etc:
    emitter.ParticlesPerSecond() = 2000;
    emitter.SetUseParticleReservoir(true);
    emitter.ParticleReservoirAmount() = 8000;

    // Our ChParticleEmitter object, among the main settings, it requires
    // that you give him four 'randomizer' objects: one is in charge of
    // generating random shapes, one is in charge of generating
    // random positions, one for random alignements, and one for random velocities.
    // In the following we need to instance such objects. (There are many ready-to-use
    // randomizer objects already available in chrono, but note that you could also
    // inherit your own class from these randomizers if the choice is not enough).

    // ---Initialize the randomizer for positions
    auto emitter_positions = chrono_types::make_shared<ChRandomParticlePositionRectangleOutlet>();
    emitter_positions->Outlet() =
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X));  // center and alignment of the outlet
    emitter_positions->OutletWidth() = 3.0;
    emitter_positions->OutletHeight() = 3.0;
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
    //    We can also mix some families of particles of different types, using different
    //    ChRandomShapeCreator  creators.

    // A)
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto mcreator_spheres = chrono_types::make_shared<ChRandomShapeCreatorSpheres>();
    mcreator_spheres->SetDiameterDistribution(
        chrono_types::make_shared<ChZhangDistribution>(0.15, 0.03));  // Zhang parameters: average val, min val.
    mcreator_spheres->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle.
    class MyCreator_spheres : public ChRandomShapeCreator::AddBodyCallback {
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> body,
                               ChCoordsys<> coords,
                               ChRandomShapeCreator& creator) override {
            body->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.4f));
            pov->SetCustomCommands(body, " texture {finish { specular 0.9 } pigment{ color rgb<0.4,0.4,0.45>} }  \n");

            // Bind the collision model to the collision system
            if (body->GetCollisionModel())
                coll->Add(body->GetCollisionModel());
        }
        ChPovRay* pov;
        ChCollisionSystem* coll;
    };
    auto callback_spheres = chrono_types::make_shared<MyCreator_spheres>();
    callback_spheres->pov = &pov_exporter;
    callback_spheres->coll = sys.GetCollisionSystem().get();
    mcreator_spheres->RegisterAddBodyCallback(callback_spheres);

    // B)
    // Create a ChRandomShapeCreator object (ex. here for box particles)

    auto creator_boxes = chrono_types::make_shared<ChRandomShapeCreatorBoxes>();
    creator_boxes->SetXsizeDistribution(
        chrono_types::make_shared<ChZhangDistribution>(0.20, 0.09));  // Zhang parameters: average val, min val.
    creator_boxes->SetSizeRatioZDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.8, 1.0));
    creator_boxes->SetSizeRatioYZDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.2, 0.3));
    creator_boxes->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1000));

    // Optional: define a callback to be exectuted at each creation of a box particle:
    class MyCreator_plastic : public ChRandomShapeCreator::AddBodyCallback {
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> body,
                               ChCoordsys<> coords,
                               ChRandomShapeCreator& creator) override {
            // Quick randomization of POV colors, without using the ChRandomShapeCreatorFromFamilies
            double icol = ChRandom();
            if (icol < 0.3)
                pov->SetCustomCommands(body, " texture {pigment{ color rgb<0.8,0.3,0.3>} }  \n");
            else if (icol < 0.8)
                pov->SetCustomCommands(body, " texture {pigment{ color rgb<0.3,0.8,0.3>} }  \n");
            else
                pov->SetCustomCommands(body, " texture {pigment{ color rgb<0.3,0.3,0.8>} }  \n");

            // Bind the collision model to the collision system
            if (body->GetCollisionModel())
                coll->Add(body->GetCollisionModel());
        }
        ChPovRay* pov;
        ChCollisionSystem* coll;
    };
    auto callback_plastic = chrono_types::make_shared<MyCreator_plastic>();
    callback_plastic->pov = &pov_exporter;
    callback_plastic->coll = sys.GetCollisionSystem().get();
    creator_boxes->RegisterAddBodyCallback(callback_plastic);

    // C)
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto creator_hulls = chrono_types::make_shared<ChRandomShapeCreatorConvexHulls>();
    creator_hulls->SetChordDistribution(
        chrono_types::make_shared<ChZhangDistribution>(0.3, 0.14));  // Zhang parameters: average val, min val.
    creator_hulls->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_hulls : public ChRandomShapeCreator::AddBodyCallback {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> body,
                               ChCoordsys<> coords,
                               ChRandomShapeCreator& creator) override {
            body->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.4f));
            pov->SetCustomCommands(body, " texture {finish { specular 0.9 } pigment{ color rgb<0.3,0.4,0.6>} }  \n");

            // Bind the collision model to the collision system
            if (body->GetCollisionModel())
                coll->Add(body->GetCollisionModel());
        }
        ChPovRay* pov;
        ChCollisionSystem* coll;
    };
    auto callback_hulls = chrono_types::make_shared<MyCreator_hulls>();
    callback_hulls->pov = &pov_exporter;
    callback_hulls->coll = sys.GetCollisionSystem().get();
    creator_hulls->RegisterAddBodyCallback(callback_hulls);

    // D)
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto creator_shavings = chrono_types::make_shared<ChRandomShapeCreatorShavings>();
    creator_shavings->SetDiameterDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.06, 0.1));
    creator_shavings->SetLengthRatioDistribution(chrono_types::make_shared<ChMinMaxDistribution>(3, 6));
    creator_shavings->SetTwistDistributionU(chrono_types::make_shared<ChMinMaxDistribution>(5, 9));
    creator_shavings->SetTwistDistributionV(chrono_types::make_shared<ChMinMaxDistribution>(2, 3));
    creator_shavings->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_shavings : public ChRandomShapeCreator::AddBodyCallback {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> body,
                               ChCoordsys<> coords,
                               ChRandomShapeCreator& creator) override {
            float scale = (float)ChRandom();
            body->GetVisualShape(0)->SetColor(ChColor(0.3f + scale * 0.6f, 0.2f + scale * 0.7f, 0.2f + scale * 0.7f));
        }
    };
    creator_shavings->RegisterAddBodyCallback(chrono_types::make_shared<MyCreator_shavings>());

    // Create a parent ChRandomShapeCreator that 'mixes' some generators above,
    // mixing them with a given percentual:

    auto creatorTot = chrono_types::make_shared<ChRandomShapeCreatorFromFamilies>();
    //    creatorTot->AddFamily(creator_metal, 1.0);    // 1st creator family, with percentual
    //    creatorTot->AddFamily(creator_boxes, 1.0);    // nth creator family, with percentual
    //    creatorTot->AddFamily(creator_hulls, 1.0);    // nth creator family, with percentual
    creatorTot->AddFamily(creator_shavings, 1.0);  // nth creator family, with percentual
    creatorTot->Setup();
    // By default, percentuals are in terms of number of generated particles,
    // but you can optionally enforce percentuals in terms of masses:
    creatorTot->SetProbabilityMode(ChRandomShapeCreatorFromFamilies::MASS_PROBABILITY);

    // Finally, tell to the emitter that it must use the 'mixer' above:
    emitter.SetParticleCreator(creatorTot);

    // --- Optional: what to do by default on ALL newly created particles?
    //     A callback executed at each particle creation can be attached to the emitter.
    //     For example, we need that new particles will be bound to Irrlicht visualization:

    // a- define a class that implement your custom OnAddBody method...
    class MyCreatorForAll : public ChRandomShapeCreator::AddBodyCallback {
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> body,
                               ChCoordsys<> coords,
                               ChRandomShapeCreator& creator) override {
            // Bind the visual model to the visualization system
            vis->BindItem(body);

            // Bind the collision model to the collision system
            if (body->GetCollisionModel())
                coll->Add(body->GetCollisionModel());

            // Enable PovRay rendering
            pov->Add(body);

            // Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
            body->SetNoGyroTorque(true);
        }

        ChVisualSystem* vis;
        ChCollisionSystem* coll;
        ChPovRay* pov;
    };

    // b- create the callback object...
    auto creation_callback = chrono_types::make_shared<MyCreatorForAll>();
    // c- set callback own data that he might need...
    creation_callback->vis = vis.get();
    creation_callback->coll = sys.GetCollisionSystem().get();
    creation_callback->pov = &pov_exporter;
    // d- attach the callback to the emitter!
    emitter.RegisterAddBodyCallback(creation_callback);

    // Bind all existing visual shapes to the visualization system
    vis->AttachSystem(&sys);

    // Export all existing visual shapes to POV-Ray
    pov_exporter.AddAll();

    // Create the .pov and .ini files for POV-Ray (this must be done only once at the beginning of the simulation).
    pov_exporter.ExportScript();

    // Modify some setting of the physical system for the simulation, if you want
    sys.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    sys.SetSolverMaxIterations(30);

    // Simulation loop
    double timestep = 0.02;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Continuosly create particle flow:
        emitter.EmitParticles(sys, timestep);

        sys.DoStepDynamics(timestep);

        // Create the incremental nnnn.dat and nnnn.pov files that will be load
        // by the pov .ini script in POV-Ray (do this at each simulation timestep)
        pov_exporter.ExportData();
    }

    return 0;
}
