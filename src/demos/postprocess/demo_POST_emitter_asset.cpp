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
// Demo code about using the ChEmitterAsset to attach an emitter to a moving
// object.
//
// =============================================================================

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/particlefactory/ChParticleRemover.h"
#include "chrono/assets/ChEmitterAsset.h"
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

// Use the main namespaces of Chrono, and other chrono namespaces
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
    ChSystemNSC mphysicalSystem;

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
//    floorBody->GetCollisionModel()->AddBox(1,12,20,ChVector<>(-5,0,0));
//    floorBody->GetCollisionModel()->AddBox(1,12,20,ChVector<>( 5,0,0));
//    floorBody->GetCollisionModel()->AddBox(10,12,1,ChVector<>(0,0,-5));
//    floorBody->GetCollisionModel()->AddBox(10,12,1,ChVector<>( 0,0,5));
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
/*
    /// Create 6 falling cubes, each with an attached emitter, this will
    /// cause six jets of particles

    for (int i=0;i<6; ++i)
    {
        double stonehenge_rad = 1;
        double phase = CH_C_2PI * ((double)i/6.0);

        // Create a rigid body :
        auto movingBody = std::make_shared<ChBodyEasyBox>(1.2, 0.4, 1.2, 3000, false);
        movingBody->SetPos(ChVector<>(stonehenge_rad*cos(phase),1,stonehenge_rad*sin(phase)));
        movingBody->SetPos_dt(ChVector<>(1.1*cos(phase),4,1.1*sin(phase)));
        movingBody->SetWvel_par(ChVector<>(2*cos(phase),2,2*sin(phase)));
        mphysicalSystem.Add(movingBody);

        auto mvisual = std::make_shared<ChColorAsset>();
        mvisual->SetColor(ChColor(1.0f, 0.5f, 0.1f));
        movingBody->AddAsset(mvisual);

        // Create a  emitter asset, that contains a ChParticleEmitter, and that will follow the body:
        auto emitter_asset = std::make_shared<ChEmitterAsset>();

        // Attach the emitter asset to the moving body:
        movingBody->AddAsset(emitter_asset);

        // Define features of the emitter:

        emitter_asset->Emitter().ParticlesPerSecond() = 3000;
        emitter_asset->Emitter().SetJitterDeclustering(true);
        emitter_asset->Emitter().SetInheritSpeed(false);
        emitter_asset->Emitter().SetUseParticleReservoir(true);
        emitter_asset->Emitter().ParticleReservoirAmount() = 5500;

 
        // ---Initialize the randomizer for positions
        auto emitter_positions = std::make_shared<ChRandomParticlePositionRectangleOutlet>();
        emitter_positions->Outlet() = ChCoordsys<>(ChVector<>(0, 0.2, 0),    // position, respect to owner moving body
                                        Q_from_AngAxis(CH_C_PI_2, VECT_X));  // rotation, respect to owner moving body
        emitter_positions->OutletWidth() = 1.2;
        emitter_positions->OutletHeight() = 1.2;
        emitter_asset->Emitter().SetParticlePositioner(emitter_positions);

        // ---Initialize the randomizer for alignments
        auto emitter_rotations = std::make_shared<ChRandomParticleAlignmentUniform>();
        emitter_asset->Emitter().SetParticleAligner(emitter_rotations);

        // ---Initialize the randomizer for velocities, with statistical distribution
        auto mvelo = std::make_shared<ChRandomParticleVelocityConstantDirection>();
        mvelo->SetDirection(VECT_Y);
        mvelo->SetModulusDistribution(5.0);
        emitter_asset->Emitter().SetParticleVelocity(mvelo);

    
        // ---Initialize a ChRandomShapeCreator object (ex. here for sphere particles)
        auto mcreator_spheres = std::make_shared<ChRandomShapeCreatorSpheres>();
        mcreator_spheres->SetDiameterDistribution(std::make_shared<ChMinMaxDistribution>(0.05, 0.15)); 
        mcreator_spheres->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1600));
        mcreator_spheres->SetAddCollisionShape(false);

        // Finally, tell to the emitter that it must use the creator above:
        emitter_asset->Emitter().SetParticleCreator(mcreator_spheres);

        // Optional: for visualization etc.
        // a- define a class that implement your custom OnAddBody method...
        class MyCreatorForAll : public ChRandomShapeCreator::AddBodyCallback {
          public:
            virtual void OnAddBody(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) override {

                // Enable Irrlicht visualization for all particles
                airrlicht_application->AssetBind(mbody);
                airrlicht_application->AssetUpdate(mbody);

                #if defined USE_POSTPROCESSING_MODULE
                    // Enable PovRay rendering
                    auto mpov_asset = std::make_shared<ChPovRayAsset>();
                    mbody->AddAsset(mpov_asset);
                
                    // Add custom POVray material..
                    auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
                    mPOVcustom->SetCommands(" texture {finish { specular 0.9 } pigment{ color rgb<0.3,0.5,0.55>} }  \n");
                    mbody->AddAsset(mPOVcustom);
                
                #endif

                // Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
                mbody->SetNoGyroTorque(true);
            }
            irrlicht::ChIrrApp* airrlicht_application;
        };
        // b- create the callback object...
        MyCreatorForAll* mcreation_callback = new MyCreatorForAll;
        // c- set callback own data that he might need...
        mcreation_callback->airrlicht_application = &application;
        // d- attach the callback to the emitter!
        emitter_asset->Emitter().RegisterAddBodyCallback(mcreation_callback);

    }
*/
    //****TEST****
int num_emitters = 5;
std::vector<ChParticleEmitter> emitters(num_emitters);
for (unsigned int ie = 0; ie < emitters.size(); ie++)
{
    // Ok, that object will take care of generating particle flows for you.
    // It accepts a lot of settings, for creating many different types of particle
    // flows, like fountains, outlets of various shapes etc.
    // For instance, set the flow rate, etc:

    emitters[ie].ParticlesPerSecond() = 3000;

    emitters[ie].SetUseParticleReservoir(true);
    emitters[ie].ParticleReservoirAmount() = 4000;

    // ---Initialize the randomizer for positions
    double xpos = (ie -0.5*num_emitters)*2.2;
    auto emitter_positions = std::make_shared<ChRandomParticlePositionRectangleOutlet>();
    emitter_positions->Outlet() =
        ChCoordsys<>(ChVector<>(xpos, -4, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X));  // center and alignment of the outlet
    emitter_positions->OutletWidth() = 1.2;
    emitter_positions->OutletHeight() = 1.2;
    emitters[ie].SetParticlePositioner(emitter_positions);

    // just for visualizing outlet
    auto boxbody = std::make_shared<ChBodyEasyBox>(1.2, 0.4, 1.2, 3000, false);
    boxbody->SetPos(ChVector<>(xpos,-4.1,0));
    boxbody->SetBodyFixed(true);
    mphysicalSystem.Add(boxbody);
    auto mvisual = std::make_shared<ChColorAsset>();
    mvisual->SetColor(ChColor(1.0f, 0.5f, 0.1f));
    boxbody->AddAsset(mvisual);

    // ---Initialize the randomizer for alignments
    auto emitter_rotations = std::make_shared<ChRandomParticleAlignmentUniform>();
    emitters[ie].SetParticleAligner(emitter_rotations);

    // ---Initialize the randomizer for velocities, with statistical distribution
    auto mvelo = std::make_shared<ChRandomParticleVelocityConstantDirection>();
    mvelo->SetDirection(VECT_Y);
    mvelo->SetModulusDistribution(8.0);

    emitters[ie].SetParticleVelocity(mvelo);

    // A)  
    // Create a ChRandomShapeCreator object (ex. here for sphere particles)

    auto mcreator_spheres = std::make_shared<ChRandomShapeCreatorSpheres>();
    mcreator_spheres->SetDiameterDistribution(std::make_shared<ChMinMaxDistribution>(0.20, 0.06));
    mcreator_spheres->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_spheres : public ChRandomShapeCreator::AddBodyCallback {
        // Here do custom stuff on the just-created particle:
      public:
          virtual void OnAddBody(std::shared_ptr<ChBody> mbody, ChCoordsys<> mcoords, ChRandomShapeCreator& mcreator) override {
            // Ex.: attach some optional assets, ex for visualization
            auto mvisual = std::make_shared<ChColorAsset>();
            mvisual->SetColor(ChColor(0.4f, 0.4f, 0.4f));
            mbody->AddAsset(mvisual);

            auto mPOVcustom = std::make_shared<ChPovRayAssetCustom>();
            mPOVcustom->SetCommands(" texture {finish { specular 0.9 } pigment{ color rgb<0.8,0.5,0.3>} }  \n");
            mbody->AddAsset(mPOVcustom);
        }
    };
    MyCreator_spheres* callback_spheres = new MyCreator_spheres;
    mcreator_spheres->RegisterAddBodyCallback(callback_spheres);

    // B)  
    // Create a ChRandomShapeCreator object (ex. here for hull particles)

    auto mcreator_hulls = std::make_shared<ChRandomShapeCreatorConvexHulls>();
    mcreator_hulls->SetChordDistribution(std::make_shared<ChMinMaxDistribution>(0.68, 0.15));
    mcreator_hulls->SetDensityDistribution(std::make_shared<ChConstantDistribution>(1600));

    // Optional: define a callback to be exectuted at each creation of a sphere particle:
    class MyCreator_hulls : public ChRandomShapeCreator::AddBodyCallback {
        // Here do custom stuff on the just-created particle:
      public:
        virtual void OnAddBody(std::shared_ptr<ChBody> mbody,
                               ChCoordsys<> mcoords,
                               ChRandomShapeCreator& mcreator) override {
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
    mcreator_hulls->RegisterAddBodyCallback(callback_hulls);


    // Create a parent ChRandomShapeCreator that 'mixes' some generators above,
    // mixing them with a given percentual:

    auto mcreatorTot = std::make_shared<ChRandomShapeCreatorFromFamilies>();
    mcreatorTot->AddFamily(mcreator_spheres,     (double)ie/(double)(num_emitters-1));    // 1st creator family, with percentual
    mcreatorTot->AddFamily(mcreator_hulls,   1.0-(double)ie/(double)(num_emitters-1));    // nth creator family, with percentual
    mcreatorTot->Setup();

    // Finally, tell to the emitter that it must use the 'mixer' above:
    emitters[ie].SetParticleCreator(mcreatorTot);

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
    emitters[ie].RegisterAddBodyCallback(mcreation_callback);
}

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



    application.SetTimestep(0.01);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene(true, true, SColor(255, 140, 161, 192));

        application.DrawAll();

        // Continuosly create particle flow:
        for (unsigned int ie = 0; ie < emitters.size(); ie++) {
            double tstart = ((double)ie/(double)num_emitters) *1;
            double tend   = tstart + 0.3;
            ChFunction_Sigma mfuns(3000, tstart,tend); 
            emitters[ie].ParticlesPerSecond() = mfuns.Get_y(application.GetSystem()->GetChTime());
            emitters[ie].EmitParticles(mphysicalSystem, application.GetTimestep());
            //GetLog() << ie << "  " << tstart << " " << mfuns.Get_y(application.GetSystem()->GetChTime()) << " " << emitters[ie].ParticlesPerSecond() << "\n";
        }

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
