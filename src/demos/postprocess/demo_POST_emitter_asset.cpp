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
// Demo code about using particle emitters
//
// =============================================================================

#include "chrono/assets/ChTexture.h"
#include "chrono/particlefactory/ChParticleEmitter.h"
#include "chrono/particlefactory/ChParticleRemover.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_postprocess/ChPovRay.h"

#include "chrono_thirdparty/filesystem/path.h"

// Use the main namespaces of Chrono, and other chrono namespaces
using namespace chrono;
using namespace chrono::particlefactory;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Create a Chrono system
    ChSystemNSC sys;

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("Particle emitter: creation from various distributions");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector<>(0, 4, -6), ChVector<>(0, -2, 0));

    // Create an exporter to POVray
    ChPovRay pov_exporter = ChPovRay(&sys);

    // Important: set the path to the template:
    pov_exporter.SetTemplateFile(GetChronoDataFile("_template_POV.pov"));

    // Set the path where it will save all .pov, .ini, .asset and .dat files
    pov_exporter.SetBasePath(GetChronoOutputPath() + "DEMO_EMITTER");

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

    floor_body->GetCollisionModel()->ClearModel();
    floor_body->GetCollisionModel()->AddBox(floor_mat, 10, 0.5, 10);
    //    floor_body->GetCollisionModel()->AddBox(floor_mat, 1,12,20,ChVector<>(-5,0,0));
    //    floor_body->GetCollisionModel()->AddBox(floor_mat, 1,12,20,ChVector<>( 5,0,0));
    //    floor_body->GetCollisionModel()->AddBox(floor_mat, 10,12,1,ChVector<>(0,0,-5));
    //    floor_body->GetCollisionModel()->AddBox(floor_mat, 10,12,1,ChVector<>( 0,0,5));
    floor_body->GetCollisionModel()->BuildModel();

    // Custom rendering in POVray:
    pov_exporter.SetCustomCommands(floor_body, "texture{ pigment{ color rgb<1,1,1>}} \n\
                             texture{ Raster(4, 0.02, rgb<0.8,0.8,0.8>) } \n\
                             texture{ Raster(4, 0.02, rgb<0.8,0.8,0.8>) rotate<0,90,0> } \n\
                             texture{ Raster(4*0.2, 0.04, rgb<0.8,0.8,0.8>) } \n\
                             texture{ Raster(4*0.2, 0.04, rgb<0.8,0.8,0.8>) rotate<0,90,0> } \n\
                              ");

    sys.Add(floor_body);

    int num_emitters = 5;
    std::vector<ChParticleEmitter> emitters(num_emitters);
    for (unsigned int ie = 0; ie < emitters.size(); ie++) {
        // Ok, that object will take care of generating particle flows for you.
        // It accepts a lot of settings, for creating many different types of particle
        // flows, like fountains, outlets of various shapes etc.
        // For instance, set the flow rate, etc:

        emitters[ie].ParticlesPerSecond() = 3000;

        emitters[ie].SetUseParticleReservoir(true);
        emitters[ie].ParticleReservoirAmount() = 4000;

        // ---Initialize the randomizer for positions
        double xpos = (ie - 0.5 * num_emitters) * 2.2;
        auto emitter_positions = chrono_types::make_shared<ChRandomParticlePositionRectangleOutlet>();
        emitter_positions->Outlet() = ChCoordsys<>(
            ChVector<>(xpos, -4, 0), Q_from_AngAxis(CH_C_PI_2, VECT_X));  // center and alignment of the outlet
        emitter_positions->OutletWidth() = 1.2;
        emitter_positions->OutletHeight() = 1.2;
        emitters[ie].SetParticlePositioner(emitter_positions);

        // just for visualizing outlet
        auto boxbody = chrono_types::make_shared<ChBodyEasyBox>(1.2, 0.4, 1.2, 3000, true, false);
        boxbody->SetPos(ChVector<>(xpos, -4.1, 0));
        boxbody->SetBodyFixed(true);
        boxbody->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.5f, 0.1f));
        sys.Add(boxbody);

        // ---Initialize the randomizer for alignments
        auto emitter_rotations = chrono_types::make_shared<ChRandomParticleAlignmentUniform>();
        emitters[ie].SetParticleAligner(emitter_rotations);

        // ---Initialize the randomizer for velocities, with statistical distribution
        auto mvelo = chrono_types::make_shared<ChRandomParticleVelocityConstantDirection>();
        mvelo->SetDirection(VECT_Y);
        mvelo->SetModulusDistribution(8.0);

        emitters[ie].SetParticleVelocity(mvelo);

        // A)
        // Create a ChRandomShapeCreator object (ex. here for sphere particles)

        auto creator_spheres = chrono_types::make_shared<ChRandomShapeCreatorSpheres>();
        creator_spheres->SetDiameterDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.20, 0.06));
        creator_spheres->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));

        // Optional: define a callback to be exectuted at each creation of a sphere particle:
        class MyCreator_spheres : public ChRandomShapeCreator::AddBodyCallback {
            // Here do custom stuff on the just-created particle:
          public:
            virtual void OnAddBody(std::shared_ptr<ChBody> body,
                                   ChCoordsys<> coords,
                                   ChRandomShapeCreator& creator) override {
                body->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.4f));
                pov->SetCustomCommands(body, " texture {finish { specular 0.9 } pigment{ color rgb<0.8,0.5,0.3>} }\n");
            }
            ChPovRay* pov;
        };
        auto callback_spheres = chrono_types::make_shared<MyCreator_spheres>();
        callback_spheres->pov = &pov_exporter;
        creator_spheres->RegisterAddBodyCallback(callback_spheres);

        // B)
        // Create a ChRandomShapeCreator object (ex. here for hull particles)

        auto creator_hulls = chrono_types::make_shared<ChRandomShapeCreatorConvexHulls>();
        creator_hulls->SetChordDistribution(chrono_types::make_shared<ChMinMaxDistribution>(0.68, 0.15));
        creator_hulls->SetDensityDistribution(chrono_types::make_shared<ChConstantDistribution>(1600));

        // Optional: define a callback to be exectuted at each creation of a sphere particle:
        class MyCreator_hulls : public ChRandomShapeCreator::AddBodyCallback {
            // Here do custom stuff on the just-created particle:
          public:
            virtual void OnAddBody(std::shared_ptr<ChBody> body,
                                   ChCoordsys<> coords,
                                   ChRandomShapeCreator& creator) override {
                body->GetVisualShape(0)->SetColor(ChColor(0.4f, 0.4f, 0.4f));
                pov->SetCustomCommands(body, " texture {finish { specular 0.9 } pigment{ color rgb<0.3,0.4,0.6>} }\n");
            }
            ChPovRay* pov;
        };
        auto callback_hulls = chrono_types::make_shared<MyCreator_hulls>();
        callback_hulls->pov = &pov_exporter;
        creator_hulls->RegisterAddBodyCallback(callback_hulls);

        // Create a parent ChRandomShapeCreator that 'mixes' some generators above,
        // mixing them with a given percentual:

        auto mcreatorTot = chrono_types::make_shared<ChRandomShapeCreatorFromFamilies>();
        mcreatorTot->AddFamily(creator_spheres,
                               (double)ie / (double)(num_emitters - 1));  // 1st creator family, with percentual
        mcreatorTot->AddFamily(creator_hulls,
                               1.0 - (double)ie / (double)(num_emitters - 1));  // nth creator family, with percentual
        mcreatorTot->Setup();

        // Finally, tell to the emitter that it must use the 'mixer' above:
        emitters[ie].SetParticleCreator(mcreatorTot);

        // --- Optional: what to do by default on ALL newly created particles?
        //     A callback executed at each particle creation can be attached to the emitter.
        //     For example, we need that new particles will be bound to Irrlicht visualization:

        // a- define a class that implement your custom OnAddBody method...
        class MyCreatorForAll : public ChRandomShapeCreator::AddBodyCallback {
          public:
            virtual void OnAddBody(std::shared_ptr<ChBody> body,
                                   ChCoordsys<> coords,
                                   ChRandomShapeCreator& creator) override {
                // Enable Irrlicht visualization for all particles
                vis->BindItem(body);

                // Enable PovRay rendering
                pov->Add(body);

                // Other stuff, ex. disable gyroscopic forces for increased integrator stabilty
                body->SetNoGyroTorque(true);
            }
            ChVisualSystemIrrlicht* vis;
            ChPovRay* pov;
        };

        // b- create the callback object...
        auto mcreation_callback = chrono_types::make_shared<MyCreatorForAll>();
        // c- set callback own data that he might need...
        mcreation_callback->vis = vis.get();
        mcreation_callback->pov = &pov_exporter;
        // d- attach the callback to the emitter!
        emitters[ie].RegisterAddBodyCallback(mcreation_callback);
    }

    // Bind all existing visual shapes to the visualization system
    vis->AttachSystem(&sys);

    // Export all existing visual shapes to POV-Ray
    pov_exporter.AddAll();

    // Create the .pov and .ini files for POV-Ray (this must be done only once at the beginning of the simulation)
    pov_exporter.ExportScript();

    // Simulation loop
    double timestep = 0.01;
    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Continuosly create particle flow
        for (unsigned int ie = 0; ie < emitters.size(); ie++) {
            double tstart = ((double)ie / (double)num_emitters) * 1;
            double tend = tstart + 0.3;
            ChFunction_Sigma mfuns(3000, tstart, tend);
            emitters[ie].ParticlesPerSecond() = mfuns.Get_y(sys.GetChTime());
            emitters[ie].EmitParticles(sys, timestep);
            // GetLog() << ie << "  " << tstart << " " << mfuns.Get_y(application.GetSystem()->GetChTime()) << " " <<
            // emitters[ie].ParticlesPerSecond() << "\n";
        }

        sys.DoStepDynamics(timestep);

        // Create the incremental nnnn.dat and nnnn.pov files that will be load
        // by the pov .ini script in POV-Ray (do this at each simulation timestep)
        pov_exporter.ExportData();
    }

    return 0;
}
