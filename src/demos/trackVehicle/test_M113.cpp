//
// PROJECT CHRONO - http://projectchrono.org
//
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
///////////////////////////////////////////////////
//
//   A tracked vehicle, M113, built and simulated using the trackedVehicle library.
//   Build the vehicle using a hierarchy of subsystems.
//   Driven by applying a motion directly to the sprocket bodies
//   Simulate by GUI input to an irrlicht EventReceiver.
//   Y-up, X-forward, Z-lateral global c-sys
//
//	 Author: Justin Madsen, 2015
///////////////////////////////////////////////////

#include "physics/ChSystem.h"
// #include "particlefactory/ChParticleEmitter.h"

#include "core/ChFileutils.h"
#include "core/ChStream.h"
#include "core/ChRealtimeStep.h"
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
#include "physics/ChBodyEasy.h"

#include "assets/ChTexture.h"
#include "assets/ChColorAsset.h"

#include "utils/ChUtilsInputOutput.h"

/*
#if IRRLICHT_ENABLED
*/
#include "chrono_irrlicht/ChIrrApp.h"
#include "subsys/driver/ChIrrGuiTrack.h"
// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
/*
# define USE_IRRLICHT
#endif
*/
#include "subsys/trackVehicle/TrackVehicleM113.h"
#include "subsys/driver/Track_FuncDriver.h"
#include "ModelDefs.h"
// Use the main namespace of Chrono
using namespace chrono;
using namespace chrono::irrlicht;


// =============================================================================
// User Settings
// =============================================================================

// *****  User set model parameters, initial conditions
const double tensioner_preload = 2.8e5;  // idler subsystem tensioner preload [N]
const double pin_damping_coef = 0.1;  // apply pin damping between connected shoes

// Initial vehicle position and heading. Defines the REF frame for the hull body
const ChVector<> initLoc(0, 0.7, 0);
// ChQuaternion<> initRot = Q_from_AngAxis(CH_C_PI_4, VECT_Y);
const ChQuaternion<> initRot(QUNIT);

// *****  Simulation step size, end time
double step_size = 1e-3;
// stop at a certain time
const double end_time = 10;  // 99999

// *****  Driver settings
// Automated simulation controls, applies positive half a sine wave.
// Otherwise, control throttle with W/S
const bool autopilot = true;
const double omega_max = 25.0;  // sprocket max rot. vel [rad/s]
const double tStart = 1.5;      // time to start applied rot. vel
const double tEnd = 7.5;        // time to reach max omega
const double ramp_slope = 1.0 / (tEnd - tStart);

// ***** write to console or a file
// #define WRITE_OUTPUT  // write output data to file
// #define CONSOLE_DEBUG_INFO   // log output data to console
// #define CONSOLE_SYSTEM_INFO  // display the system heirarchy in console
#define CONSOLE_TIMING  // timers for each render and simulation step, log to console

// for each subsystem, decide what TYPE of data to save
int debug_type = DBG_BODY | DBG_CONSTRAINTS;  //  | DBG_CONTACTS;
// vehicle subsystems and objects to save data for
int what_to_save = DBG_FIRSTSHOE | DBG_GEAR | DBG_IDLER | DBG_CHASSIS;
// vehicle subsystems and objects to write data to console
int what_to_console = DBG_FIRSTSHOE | DBG_GEAR | DBG_IDLER | DBG_CHASSIS;

const double save_step_size = 2e-3;    // Time interval for writing data to file, don't exceed 1 kHz.
const double console_step_size = 1.0;  // time interval for writing data to console
const std::string save_filename = "M113_400_200_";
const std::string save_outDir = "../outdata_M113";

// ***** flat ground size and COG location
const ChVector<> groundSize(100.0, 1.0, 100.0);
const ChVector<> groundPos(0, -1.0, 0);
double mu = 0.3;  // dry friction coef.

// *****  Visualization and camera settings
const int FPS = 80;
const double render_step_size = 1.0 / FPS;  // FPS = 50

// #ifdef USE_IRRLICHT
// camera controls, either static or  GUI controlled chase camera:
bool use_fixed_camera = false;
// static camera position, global c-sys. (Longitude, Vertical, Lateral)
ChVector<> fixed_cameraPos(2, 1.15, 3);  // (0.15, 1.15, 1.5);
// relative to center of chassis
ChVector<> trackPoint(0.5, -0.5, 0);

// Point on chassis tracked by the camera
double chaseDist = 3.5;    // 4.0;
double chaseHeight = 1.0;  // 1.0;

bool do_shadows = true;  // shadow map is experimental
                         /*
                       #else
                         double tend = 20.0;
                         
                         const std::string out_dir = "../M113";
                         const std::string pov_dir = out_dir + "/POVRAY";
                       #endif
                         */

/// the ground body, visual assets and collision shape.
ChSharedPtr<ChBody> Add_FlatGround(TrackVehicleM113* vehicle,
                                   const ChVector<>& size,
                                   const ChVector<>& pos,
                                   double friction_mu) {
    // body, visual and collision all in one.
    ChSharedPtr<ChBody> ground(new ChBodyEasyBox(size.x, size.y, size.z, 1000.0, true, true));
    ground->SetBodyFixed(true);
    ground->SetPos(pos);
    // ground->SetIdentifier(-1);
    ground->SetName("ground");
    ground->GetMaterialSurface()->SetFriction(friction_mu);

    // Don't forget to set this collision family!
    ground->GetCollisionModel()->SetFamily((int)CollisionFam::Ground);

    // color asset for the ground
    ChSharedPtr<ChColorAsset> groundColor(new ChColorAsset);
    groundColor->SetColor(ChColor(0.2f, 0.4f, 0.8f));
    // ground->AddAsset(groundColor);

    // add a texture to the ground
    ChSharedPtr<ChTexture> tex(new ChTexture);
    tex->SetTextureFilename(GetChronoDataFile("track_data/terrain/glenway.jpg"));
    ground->AddAsset(tex);

    vehicle->GetSystem()->AddBody(ground);  // add this body to the system, which is the vehicle

    // add some static obstacles
    if (0) {
        ChVector<> rampSize = size / 10.0;
        ChSharedPtr<ChBody> ramp(new ChBodyEasyBox(rampSize.x, rampSize.y, rampSize.z, 1000.0, true, true));
        ramp->SetPos(ground->GetPos());
        ramp->SetBodyFixed(true);
        ramp->SetName("ramp");
        ramp->GetMaterialSurface()->SetFriction(friction_mu);

        ramp->GetCollisionModel()->SetFamily((int)CollisionFam::Ground);

        // set color for the ramp
        ChSharedPtr<ChColorAsset> rampCol(new ChColorAsset);
        rampCol->SetColor(ChColor(0.2f, 0.4f, 0.6f));
        ramp->AddAsset(rampCol);

        vehicle->GetSystem()->AddBody(ramp);
    }

    return ground;
}

int main(int argc, char* argv[]) {
    // NOTE: utils library built with this sets this statically
    // SetChronoDataPath(CHRONO_DATA_DIR);
    // --------------------------
    // Create the tracked vehicle and the ground/environment

    // The vehicle inherits ChSystem. Input chassis visual and collision type
    TrackVehicleM113 vehicle("M113 model for validation", VisualizationType::None, CollisionType::None,
                             5489.2,                                // chass mass
                             ChVector<>(1786.9, 10449.7, 10721.2),  // chassis mass
                             pin_damping_coef,                      // iner-shoe pin damping (if any)
                             tensioner_preload, omega_max);

    // set the chassis REF at the specified initial config.
    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    // ground is a large flat plate
    // TODO: superimposed obstacles using ChParticleEmitter class.
    ChSharedPtr<ChBody> ground = Add_FlatGround(&vehicle, groundSize, groundPos, mu);

// if writing an output file, setup what debugInformation we want added each step data is saved.
#ifdef WRITE_OUTPUT
    vehicle.Setup_logger(what_to_save, debug_type, save_filename, save_outDir);
#endif

    /*
    #ifdef USE_IRRLICHT
    */
    // Setup the Irrlicht GUI
    size_t window_x_len = 1200;
    size_t window_y_len = 800;
    // Create the Irrlicht visualization applicaiton
    ChIrrApp application(vehicle.GetSystem(), L"M113 tracked vehicle demo",
                         dimension2d<u32>(window_x_len, window_y_len), false, do_shadows);
    // assumes Y-up
    application.AddTypicalSky();

    irr::scene::ILightSceneNode* mlight = 0;

    if (do_shadows) {
        mlight =
            application.AddLightWithShadow(irr::core::vector3df(10.f, 60.f, 30.f), irr::core::vector3df(0.f, 0.f, 0.f),
                                           150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
    } else {
        application.AddTypicalLights(irr::core::vector3df(30.f, 100.f, 30.f), irr::core::vector3df(30.f, 100.f, 50.f),
                                     250, 130);
    }

    application.SetTimestep(step_size);

    // the GUI driver
    ChIrrGuiTrack driver(application, vehicle, trackPoint, chaseDist, chaseHeight, window_x_len - 150);
    if (use_fixed_camera)
        driver.SetCameraPos(fixed_cameraPos);
    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // Set up the assets for rendering
    application.AssetBindAll();
    application.AssetUpdateAll();
    if (do_shadows) {
        application.AddShadowAll();
    }
    /*
    #else

    #endif
    */

    // when autopilot is enabled, input throttle specified as follows:
    ChSharedPtr<ChFunction_Ramp> ramp_func(new ChFunction_Ramp(0, ramp_slope));
    Track_FuncDriver<ChFunction_Ramp> function_driver(2, ramp_func, tStart, -1, omega_max);

    // test: using a sine function
    // double sineAmp = 0.4;
    // double sineFreq = 0.3;
    // ChSharedPtr<ChFunction_Sine> sine_func(new ChFunction_Sine(0, sineFreq, sineAmp));
    // Track_FuncDriver<ChFunction_Sine> function_driver(2, sine_func, tStart);

    // ---------------------
    // GUI and render settings

    // GUI driver inputs
    std::vector<double> throttle_input;
    std::vector<double> braking_input;
    double steering_input;

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Number of simulation steps between two output frames
    int save_steps = (int)std::ceil(save_step_size / step_size);

    // Number of steps between two log to consoles
    int console_steps = (int)std::ceil(console_step_size / step_size);

// ---------------------
// Simulation loop
#ifdef CONSOLE_SYSTEM_INFO
    GetLog() << "\n\n============ System Configuration ============\n";
    vehicle.GetSystem()->ShowHierarchy(GetLog());
#endif

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    double time = 0;
    // create some timers, for the render and total time
    ChTimer<double> step_timer;
    double total_step_time = 0;
    double time_since_last_output = 0;

    //  #ifdef USE_IRRLICHT
    // using Irrlicht? time that too.
    ChTimer<double> render_timer;
    double total_render_time = 0;

//  #endif
/*
#ifdef USE_IRRLICHT
*/

// write data to file?
#ifdef WRITE_OUTPUT
    vehicle.Log_to_file();  // needs to already be setup before sim loop calls it
#endif
    ChRealtimeStepTimer realtime_timer;
    bool is_end_time_reached = false;
    while (application.GetDevice()->run() && (!is_end_time_reached)) {
        // keep track of the time spent calculating each sim step
        step_timer.start();

        // Render scene
        if (step_number % render_steps == 0) {
            render_timer.start();  // start the time it takes to render the scene
            application.GetVideoDriver()->beginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            driver.DrawAll();
            application.GetVideoDriver()->endScene();

            render_timer.stop();                  // stop the scene timer
            total_render_time += render_timer();  // increment the time it took to render this step
        }

        //  input is specified by the user, or by a function (autopilot)
        if (autopilot) {
            throttle_input = function_driver.GetThrottle();
            braking_input = function_driver.GetBraking();
            // set the GUI info
            driver.SetThrottleFunc(0, throttle_input[0]);
            driver.SetThrottleFunc(1, throttle_input[1]);
            // driver.SetBrakingFunc(braking_input);
        } else {
            // Collect output data from modules (for inter-module communication)
            throttle_input = driver.GetThrottle();
            braking_input = driver.GetBraking();
        }

        // Update
        time = vehicle.GetSystem()->GetChTime();

        driver.Update(time);

        if (autopilot)
            function_driver.Update(time);

        vehicle.Update(time, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        // double step = realtime_timer.SuggestSimulationStep(step_size);
        use_fixed_camera ? driver.Advance(step_size, fixed_cameraPos) : driver.Advance(step_size);

        if (!application.GetPaused()) {
            vehicle.Advance(step_size);
            step_number++;
        }

        // stop and increment the step timer
        step_timer.stop();
        total_step_time += step_timer();
        time_since_last_output += step_timer();

#ifdef WRITE_OUTPUT
        if (step_number % save_steps == 0) {
            // write data to file
            vehicle.Log_to_file();  // needs to already be setup before sim loop calls it
        }
#endif

        if (step_number % console_steps == 0) {
// log desired output to console
#ifdef CONSOLE_DEBUG_INFO
            vehicle.Log_to_console(what_to_console);
#endif

#ifdef CONSOLE_TIMING
            GetLog() << "\n --------- TIMING -------- : time: " << vehicle.GetSystem()->GetChTime()
                     << "\n total render time (ms): " << total_render_time
                     << ",  % of total: " << 100. * total_render_time / total_step_time
                     << "\n total compute time (sec): " << total_step_time / 1000.0 << "\n Avg. time per step (ms): "
                     << time_since_last_output * vehicle.GetSystem()->GetStep() / save_steps
                     << "\n overall avg. time/step (ms): " << total_step_time / step_number / vehicle.GetSystem()->GetChTime()
                     << "    for a stepsize (ms): " << vehicle.GetSystem()->GetStep() * 1000.0
                     << "\n RTR : " << total_step_time / vehicle.GetSystem()->GetChTime() / 1000.0;
            time_since_last_output = 0;
#endif
        }

        // see if the end time is reached
        if (time > end_time)
            is_end_time_reached = true;

    }  // end simulation loop

    application.GetDevice()->drop();

    /*
    #else

      int render_frame = 0;

      if(ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
      }
      if(ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
      }

      vehicle.ExportMeshPovray(out_dir);

      char filename[100];

      while (time < tend)
      {
        if (step_number % render_steps == 0) {
          // Output render data
          sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
          utils::WriteShapesPovray(vehicle.GetSystem(), filename);
          std::cout << "Output frame:   " << render_frame << std::endl;
          std::cout << "Sim frame:      " << step_number << std::endl;
          std::cout << "Time:           " << time << std::endl;
          std::cout << " throttle: " << driver.GetThrottle() << " steering: " << driver.GetSteering() << std::endl;
          std::cout << std::endl;
          render_frame++;
        }

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();

        driver.Update(time);

        vehicle.Update(time, steering_input, braking_input, powertrain_torque, tire_forces);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);

        vehicle.Advance(step_size);

        // Increment frame number
        step_number++;
      }

    #endif

    */

    return 0;
}
