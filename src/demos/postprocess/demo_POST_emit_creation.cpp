// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

#include "chrono/physics/ChSystem.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/particlefactory/ChParticleRemover.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/core/ChFileutils.h"

#include "chrono_irrlicht/ChIrrApp.h"

#define USE_POSTPROCESSING_MODULE

#if defined USE_POSTPROCESSING_MODULE
#include "chrono_postprocess/ChPovRay.h"
#include "chrono_postprocess/ChPovRayAsset.h"
#include "chrono_postprocess/ChPovRayAssetCustom.h"
using namespace chrono::postprocess;
#endif

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
    // Create a ChronoENGINE physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"Particle emitter: creation from various distributions", core::dimension2d<u32>(800, 600),
                         false);

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    ChIrrWizard::add_typical_Logo(application.GetDevice());
    ChIrrWizard::add_typical_Sky(application.GetDevice());
    ChIrrWizard::add_typical_Lights(application.GetDevice());
    ChIrrWizard::add_typical_Camera(application.GetDevice(), core::vector3df(0, 4, -6), core::vector3df(0, -2, 0));

    //
    // CREATE THE SYSTEM OBJECTS
    //

    // Create the floor:

    auto floorBody = std::make_shared<ChBodyEasyBox>(20, 1, 20, 1000, true, true);
    floorBody->SetPos(ChVector<>(0, -5, 0));
    floorBody->SetBodyFixed(true);
    floorBody->GetCollisionModel()->ClearModel();
    floorBody->GetCollisionModel()->AddBox(10,0.5,10);
    floorBody->GetCollisionModel()->AddBox(1,12,20,ChVector<>(-5,0,0));
    floorBody->GetCollisionModel()->AddBox(1,12,20,ChVector<>( 5,0,0));
    floorBody->GetCollisionModel()->AddBox(10,12,1,ChVector<>(0,0,-5));
    floorBody->GetCollisionModel()->AddBox(10,12,1,ChVector<>( 0,0,5));
    floorBody->GetCollisionModel()->BuildModel();

    auto mvisual = std::make_shared<ChColorAsset>();
    mvisual->SetColor(ChColor(0.0f, 1.0f, (float)ChRandom()));
    floorBody->AddAsset(mvisual);

    #if defined USE_POSTPROCESSING_MODULE
    // Custom rendering in POVray:
    auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
    mPOVcustom->SetCommands("texture{ pigment{ color rgb<1,1,1>}} \n\
                             texture{ Raster(4, 0.02, rgb<0.8,0.8,0.8>) } \n\
                             texture{ Raster(4, 0.02, rgb<0.8,0.8,0.8>) rotate<0,90,0> } \n\
                             texture{ Raster(4*0.2, 0.04, rgb<0.8,0.8,0.8>) } \n\
                             texture{ Raster(4*0.2, 0.04, rgb<0.8,0.8,0.8>) rotate<0,90,0> } \n\
                              ");
    floorBody->AddAsset(mPOVcustom);

     // Attach asset for marking it as renderable in PovRay
    auto mpov_asset = std::make_shared<ChPovRayAsset>();
    floorBody->AddAsset(mpov_asset);
    #endif

    mphysicalSystem.Add(floorBody);

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
    auto emitter_positions = std::make_shared<ChRandomParticlePositionRectangleOutlet>();
    emitter_positions->Outlet() =
        ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X));  // center and alignment of the outlet
    emitter_positions->OutletWidth() = 3.0;
    emitter_positions->OutletHeight() = 3.0;
    emitter.SetParticlePositioner(emitter_positions);

    // ---Initialize the randomizer for alignments
    auto emitter_rotations = std::make_shared<ChRandomParticleAlignmentUniform>();
    emitter.SetParticleAligner(emitter_rotations);

    // ---Initialize the randomizer for velocities, with statistical distribution
    auto mvelo = std::make_shared<ChRandomParticleVelocityConstantDirection>();
    mvelo->SetDirection(-VECT_Y);
    mvelo->SetModulusDistribution(0.0);

    emitter.SetParticleVelocity(mvelo);

    // ---Initialize the randomizer for creations, with statistical distribution
    //    We can also mix some families of particles of different types, using different
    //    ChRandomShapeCreator  creators.


    // A)  
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto mcreator_spheres = std::make_shared<ChRandomShapeCreatorSpheres>();
    mcreator_spheres->SetDiameterDistribution(std::make_shared<ChZhangDistribution>(0.15, 0.03));  // Zhang parameters: average val, min val.
    mcreator_spheres->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_spheres : public ChCallbackPostCreation {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) {
            // Ex.: attach some optional assets, ex for visualization
            auto mvisual = std::make_shared<ChColorAsset>();
            mvisual->SetColor(ChColor(0.4f, 0.4f, 0.4f));
            mbody->AddAsset(mvisual);

            auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
            mPOVcustom->SetCommands(" texture {finish { specular 0.9 } pigment{ color rgb<0.4,0.4,0.45>} }  \n");
            mbody->AddAsset(mPOVcustom);
        }
    };
    MyCreator_spheres* callback_spheres = new MyCreator_spheres;
    mcreator_spheres->SetCallbackPostCreation(callback_spheres);


    // B)
    // Create a ChRandomShapeCreator object (ex. here for box particles)

    auto mcreator_boxes = std::make_shared<ChRandomShapeCreatorBoxes>();
    mcreator_boxes->SetXsizeDistribution(std::make_shared<ChZhangDistribution>(0.20, 0.09));  // Zhang parameters: average val, min val.
    mcreator_boxes->SetSizeRatioZDistribution(std::make_shared<ChMinMaxDistribution>(0.8, 1.0));
    mcreator_boxes->SetSizeRatioYZDistribution(std::make_shared<ChMinMaxDistribution>(0.2, 0.3));
    mcreator_boxes->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1000));

    // Optional: define a callback to be exectuted at each creation of a box particle:
    class MyCreator_plastic : public ChCallbackPostCreation {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) {
            // Ex.: attach some optional assets, ex for visualization
            // Here do a quick randomization of POV colors, without using the ChRandomShapeCreatorFromFamilies
            auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
            mbody->AddAsset(mPOVcustom);

            double icol= ChRandom();
            if (icol < 0.3)
                mPOVcustom->SetCommands(" texture {pigment{ color rgb<0.8,0.3,0.3>} }  \n");
            else if (icol < 0.8)
                mPOVcustom->SetCommands(" texture {pigment{ color rgb<0.3,0.8,0.3>} }  \n");
            else
                mPOVcustom->SetCommands(" texture {pigment{ color rgb<0.3,0.3,0.8>} }  \n");
        }
    };
    MyCreator_plastic* callback_boxes = new MyCreator_plastic;
    mcreator_boxes->SetCallbackPostCreation(callback_boxes);


    // C)  
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto mcreator_hulls = std::make_shared<ChRandomShapeCreatorConvexHulls>();
    mcreator_hulls->SetChordDistribution(std::make_shared<ChZhangDistribution>(0.3, 0.14));  // Zhang parameters: average val, min val.
    mcreator_hulls->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_hulls : public ChCallbackPostCreation {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) {
            // Ex.: attach some optional assets, ex for visualization
            auto mvisual = std::make_shared<ChColorAsset>();
            mvisual->SetColor(ChColor(0.4f, 0.4f, 0.4f));
            mbody->AddAsset(mvisual);

            auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
            mPOVcustom->SetCommands(" texture {finish { specular 0.9 } pigment{ color rgb<0.3,0.4,0.6>} }  \n");
            mbody->AddAsset(mPOVcustom);
        }
    };
    MyCreator_hulls* callback_hulls = new MyCreator_hulls;
    mcreator_hulls->SetCallbackPostCreation(callback_hulls);


    // D)  
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto mcreator_shavings = std::make_shared<ChRandomShapeCreatorShavings>();
    mcreator_shavings->SetDiameterDistribution(std::make_shared<ChMinMaxDistribution>(0.06, 0.1));
    mcreator_shavings->SetLengthRatioDistribution(std::make_shared<ChMinMaxDistribution>(3, 6));
    mcreator_shavings->SetTwistDistributionU(std::make_shared<ChMinMaxDistribution>(5, 9));
    mcreator_shavings->SetTwistDistributionV(std::make_shared<ChMinMaxDistribution>(2, 3));
    mcreator_shavings->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_shavings : public ChCallbackPostCreation {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) {
            // Ex.: attach some optional assets, ex for visualization
            float acolscale = (float)ChRandom();
            auto mvisual = std::make_shared<ChColorAsset>();
            mvisual->SetColor(ChColor(0.3f+acolscale*0.6f, 0.2f+acolscale*0.7f, 0.2f+acolscale*0.7f));
            mbody->AddAsset(mvisual);
        }
    };
    MyCreator_shavings* callback_shavings = new MyCreator_shavings;
    mcreator_shavings->SetCallbackPostCreation(callback_shavings);


    // Create a parent ChRandomShapeCreator that 'mixes' some generators above,
    // mixing them with a given percentual:

    auto mcreatorTot = std::make_shared<ChRandomShapeCreatorFromFamilies>();
//    mcreatorTot->AddFamily(mcreator_metal, 1.0);    // 1st creator family, with percentual
//    mcreatorTot->AddFamily(mcreator_boxes, 1.0);    // nth creator family, with percentual
//    mcreatorTot->AddFamily(mcreator_hulls, 1.0);    // nth creator family, with percentual
      mcreatorTot->AddFamily(mcreator_shavings, 1.0); // nth creator family, with percentual
    mcreatorTot->Setup();
    // By default, percentuals are in terms of number of generated particles,
    // but you can optionally enforce percentuals in terms of masses:
    mcreatorTot->SetProbabilityMode(ChRandomShapeCreatorFromFamilies::MASS_PROBABILITY);


    // Finally, tell to the emitter that it must use the 'mixer' above:
    emitter.SetParticleCreator(mcreatorTot);

    // --- Optional: what to do by default on ALL newly created particles?
    //     A callback executed at each particle creation can be attached to the emitter.
    //     For example, we need that new particles will be bound to Irrlicht visualization:

    // a- define a class that implement your custom PostCreation method...
    class MyCreatorForAll : public ChCallbackPostCreation {
      public:
        virtual void PostCreation(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) {
            // Enable Irrlicht visualization for all particles
            airrlicht_application->AssetBind(mbody);
            airrlicht_application->AssetUpdate(mbody);

            // Enable PovRay rendering
            #if defined USE_POSTPROCESSING_MODULE
            auto mpov_asset = std::make_shared<ChPovRayAsset>();
            mbody->AddAsset(mpov_asset);
            #endif

            // Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
            mbody->SetNoGyroTorque(true);
        }
        ChIrrApp* airrlicht_application;
    };
    // b- create the callback object...
    MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
    // c- set callback own data that he might need...
    mcreation_callback->airrlicht_application = &application;
    // d- attach the callback to the emitter!
    emitter.SetCallbackPostCreation(mcreation_callback);


    // Use this function for adding a ChIrrNodeAsset to all already created items (ex. the floor, etc.)
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();


    #if defined USE_POSTPROCESSING_MODULE

    // Create an exporter to POVray !!
    ChPovRay pov_exporter = ChPovRay(&mphysicalSystem);

    // Sets some file names for in-out processes.
    pov_exporter.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));
    pov_exporter.SetOutputScriptFile("rendering_frames.pov");
    pov_exporter.SetOutputDataFilebase("my_state");
    pov_exporter.SetPictureFilebase("picture");

    // Even better: save the .dat files and the .bmp files
    // in two subdirectories, to avoid cluttering the current
    // directory...
    ChFileutils::MakeDirectory("output");
    ChFileutils::MakeDirectory("anim");

    pov_exporter.SetOutputDataFilebase("output/my_state");
    pov_exporter.SetPictureFilebase("anim/picture");

    pov_exporter.SetLight(VNULL,ChColor(0,0,0),false);
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

    // Use this function for adding a ChPovRayAsset to all already created items (ex. the floor, etc.)
    // Otherwise add a ChPovRayAsset on a per-item basis
    pov_exporter.AddAll();

    // 1) Create the two .pov and .ini files for POV-Ray (this must be done
    //    only once at the beginning of the simulation).
    pov_exporter.ExportScript();

    #endif


    // Modify some setting of the physical system for the simulation, if you want
    mphysicalSystem.SetSolverType(ChSolver::Type::BARZILAIBORWEIN);
    mphysicalSystem.SetMaxItersSolverSpeed(30);
    mphysicalSystem.SetMaxItersSolverStab(5);

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

        // Create the incremental nnnn.dat and nnnn.pov files that will be load
        // by the pov .ini script in POV-Ray (do this at each simulation timestep)
        #if defined USE_POSTPROCESSING_MODULE
          pov_exporter.ExportData();
        #endif
    }

    return 0;
}
