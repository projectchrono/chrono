//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//
//   Demo code about
//
//     - using the ChParticleEmitter to create
//       a cluster of random shapes
//     - apply custom force field to particles
//     - use Irrlicht to display objects
//
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono/physics/ChSystem.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_irrlicht/ChIrrApp.h"

// Use the main namespace of Chrono, and other chrono namespaces

using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::geometry;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

//     A callback executed at each particle creation can be attached to the emitter.
//     For example, we need that new particles will be bound to Irrlicht visualization:

class MyCreatorForAll : public ChCallbackPostCreation {
  public:
    virtual void PostCreation(ChSharedPtr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) {
        // optional: add further assets, ex for improving visualization:
        ChSharedPtr<ChTexture> mtexture(new ChTexture());
        mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
        mbody->AddAsset(mtexture);

        // Enable Irrlicht visualization for all particles
        airrlicht_application->AssetBind(mbody);
        airrlicht_application->AssetUpdate(mbody);

        // Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
        mbody->SetNoGyroTorque(true);
    }
    irr::ChIrrApp* airrlicht_application;
};

int main(int argc, char* argv[]) {
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Particle emitter", core::dimension2d<u32>(800, 600), false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 14, -20));

    //
    // CREATE THE SYSTEM OBJECTS
    //

    // Example: create a ChBody rigid body (trick: using the ChBodyEasyXXYYZZ
    // functions it also sets mass and inertia tensor for you, and collision
    // and visualization shapes are added automatically)

    ChSharedPtr<ChBodyEasySphere> msphereBody(new ChBodyEasySphere(2.1,     // radius size
                                                                   1800,    // density
                                                                   true,    // collide enable?
                                                                   true));  // visualization?
    msphereBody->SetPos(ChVector<>(1, 1, 0));
    msphereBody->GetMaterialSurface()->SetFriction(0.2f);

    // optional: add further assets, ex for improving visualization:
    ChSharedPtr<ChTexture> mtexture(new ChTexture());
    mtexture->SetTextureFilename(GetChronoDataFile("concrete.jpg"));
    msphereBody->AddAsset(mtexture);

    mphysicalSystem.Add(msphereBody);

    // Ok, creating particles using ChBody or the ChBodyEasyXXYYZZ shortcuts
    // can be enough, ex. if you put in a for() loop you can create a cluster.
    // However there is an easier way to the creation of cluster of particles,
    // namely the ChEmitter helper class. Here we show hof to use it.

    // Create an emitter:

    ChParticleEmitter emitter;

    // Ok, that object will take care of generating particle flows for you.
    // It accepts a lot of settings, for creating many different types of particle
    // flows, like fountains, outlets of various shapes etc.
    // For instance, set the flow rate, etc:

    emitter.ParticlesPerSecond() = 2000;

    emitter.SetUseParticleReservoir(true);
    emitter.ParticleReservoirAmount() = 200;

    // Our ChParticleEmitter object, among the main settings, it requires
    // that you give him four 'randomizer' objects: one is in charge of
    // generating random shapes, one is in charge of generating
    // random positions, one for random alignements, and one for random velocities.
    // In the following we need to instance such objects. (There are many ready-to-use
    // randomizer objects already available in chrono, but note that you could also
    // inherit your own class from these randomizers if the choice is not enough).

    // ---Initialize the randomizer for POSITIONS: random points in a large cube
    ChSharedPtr<ChRandomParticlePositionOnGeometry> emitter_positions(new ChRandomParticlePositionOnGeometry);

    ChSmartPtr<ChGeometry> sampled_cube(new ChBox(VNULL, ChMatrix33<>(QUNIT), ChVector<>(50, 50, 50)));
    emitter_positions->SetGeometry(sampled_cube);

    emitter.SetParticlePositioner(emitter_positions);

    // ---Initialize the randomizer for ALIGNMENTS
    ChSharedPtr<ChRandomParticleAlignmentUniform> emitter_rotations(new ChRandomParticleAlignmentUniform);
    emitter.SetParticleAligner(emitter_rotations);

    // ---Initialize the randomizer for VELOCITIES, with statistical distribution
    ChSharedPtr<ChRandomParticleVelocityAnyDirection> mvelo(new ChRandomParticleVelocityAnyDirection);
    mvelo->SetModulusDistribution(ChSmartPtr<ChMinMaxDistribution>(new ChMinMaxDistribution(0.0, 0.05)));
    emitter.SetParticleVelocity(mvelo);

    // ---Initialize the randomizer for ANGULAR VELOCITIES, with statistical distribution
    ChSharedPtr<ChRandomParticleVelocityAnyDirection> mangvelo(new ChRandomParticleVelocityAnyDirection);
    mangvelo->SetModulusDistribution(ChSmartPtr<ChMinMaxDistribution>(new ChMinMaxDistribution(0.0, 0.2)));
    emitter.SetParticleAngularVelocity(mangvelo);

    // ---Initialize the randomizer for CREATED SHAPES, with statistical distribution
    /*
         // Create a ChRandomShapeCreator object (ex. here for sphere particles)
        ChSharedPtr<ChRandomShapeCreatorSpheres> mcreator_spheres(new ChRandomShapeCreatorSpheres);
        mcreator_spheres->SetDiameterDistribution( ChSmartPtr<ChZhangDistribution>  (new ChZhangDistribution(0.6, 0.23))
       );
        mcreator_spheres->SetDensityDistribution ( ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1600))
       );

         // Finally, tell to the emitter that it must use the creator above:
        emitter.SetParticleCreator(mcreator_spheres);
    */

    // ..as an alternative: create odd shapes with convex hulls, like faceted fragments:
    ChSharedPtr<ChRandomShapeCreatorConvexHulls> mcreator_hulls(new ChRandomShapeCreatorConvexHulls);
    mcreator_hulls->SetNpoints(15);
    mcreator_hulls->SetChordDistribution(ChSmartPtr<ChZhangDistribution>(new ChZhangDistribution(1.3, 0.4)));
    mcreator_hulls->SetDensityDistribution(ChSmartPtr<ChConstantDistribution>(new ChConstantDistribution(1600)));
    emitter.SetParticleCreator(mcreator_hulls);

    // --- Optional: what to do by default on ALL newly created particles?
    //     A callback executed at each particle creation can be attached to the emitter.
    //     For example, we need that new particles will be bound to Irrlicht visualization:

    // a- define a class that implement your custom PostCreation method (see top of source file)
    // b- create the callback object...
    MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
    // c- set callback own data that he might need...
    mcreation_callback->airrlicht_application = &application;
    // d- attach the callback to the emitter!
    emitter.SetCallbackPostCreation(mcreation_callback);

    // Use this function for adding a ChIrrNodeAsset to all already created items (ex. a floor, a wall, etc.)
    // Otherwise use application.AssetBind(myitem); on a per-item basis, as in the creation callback.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetLcpSolverType(ChSystem::LCP_ITERATIVE_SOR);  // try also other ChSystem::LCP_XXXYYYZZZ stuff
    mphysicalSystem.SetIterLCPmaxItersSpeed(40);
    mphysicalSystem.SetIterLCPmaxItersStab(5);

    // Turn off default -9.8 downward gravity
    mphysicalSystem.Set_G_acc(ChVector<>(0, 0, 0));

    application.SetStepManage(true);
    application.SetTimestep(0.01);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.GetVideoDriver()->beginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        // Create particle flow
        emitter.EmitParticles(mphysicalSystem, application.GetTimestep());

        // Apply custom forcefield (brute force approach..)
        // A) reset 'user forces accumulators':
        for (unsigned int i = 0; i < mphysicalSystem.Get_bodylist()->size(); i++) {
            ChBody* abody = (*mphysicalSystem.Get_bodylist())[i];
            abody->Empty_forces_accumulators();
        }

        // B) store user computed force:
        // double G_constant = 6.674e-11; // gravitational constant
        double G_constant = 6.674e-3;  // gravitational constant - HACK to speed up simulation
        for (unsigned int i = 0; i < mphysicalSystem.Get_bodylist()->size(); i++) {
            ChBody* abodyA = (*mphysicalSystem.Get_bodylist())[i];
            for (unsigned int j = i + 1; j < mphysicalSystem.Get_bodylist()->size(); j++) {
                ChBody* abodyB = (*mphysicalSystem.Get_bodylist())[j];
                ChVector<> D_attract = abodyB->GetPos() - abodyA->GetPos();
                double r_attract = D_attract.Length();
                double f_attract = G_constant * (abodyA->GetMass() * abodyB->GetMass()) / (pow(r_attract, 2));
                ChVector<> F_attract = (D_attract / r_attract) * f_attract;

                abodyA->Accumulate_force(F_attract, abodyA->GetPos(), false);
                abodyB->Accumulate_force(-F_attract, abodyB->GetPos(), false);
            }
        }

        // Perform the integration timestep
        application.DoStep();

        application.GetVideoDriver()->endScene();
    }

    return 0;
}
