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
// Authors: Radu Serban
// =============================================================================
//
// Main driver function for a vehicle specified through JSON files.
//
// If using the Irrlicht interface, driver inputs are obtained from the keyboard.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include <vector>

#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkDistance.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChFilters.h"

#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/ShaftsPowertrain.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/FialaTire.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChPacejkaTire.h"
#include "lcp/ChLcpIterativeSolver.h"

#include "chrono_vehicle/ChConfigVehicle.h"

// If Irrlicht support is available...
#ifdef CHRONO_IRRLICHT
// ...include additional headers
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

// ...and specify whether the demo should actually use Irrlicht
#define USE_IRRLICHT
#endif

using namespace chrono;
using namespace chrono::vehicle;

// =============================================================================

// JSON file for vehicle model
std::string vehicle_file("hmmwv/vehicle/HMMWV_Vehicle_4WD.json");

// JSON files for terrain (rigid plane), and powertrain (simple)
std::string rigidterrain_file("terrain/RigidPlane.json");
std::string shaftspowertrain_file("generic/powertrain/ShaftsPowertrain.json");

// JSON files tire models (rigid)
std::string rigidtire_file("hmmwv/tire/HMMWV_RigidTire.json");
std::string fialatire_file("hmmwv/tire/HMMWV_FialaTire.json");
std::string pactire_file("hmmwv/tire/HMMWV_pacejka.tir");

// Driver input file (if not using Irrlicht)
std::string driver_file("generic/driver/Sample_Maneuver.txt");

// Input file names for the path-follower driver model
std::string steering_controller_file("generic/driver/SteeringController.json");
std::string speed_controller_file("generic/driver/SpeedController.json");
std::string path_file("paths/NATO_double_lane_change.txt");

// Desired vehicle speed (m/s)
double target_speed = 15.6464; // 35 mph

// Initial vehicle position
ChVector<> initLoc(-125, -125, 0.5);

// Initial vehicle orientation
ChQuaternion<> initRot(1, 0, 0, 0);

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 500.0;  // size in X direction
double terrainWidth = 500.0;   // size in Y direction

// Simulation step size
double step_size = 1e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Time interval between two output frames
double output_step_size = 1.0 / 100;  // once a second

// Point on chassis tracked by the camera (Irrlicht only)
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Simulation length (Povray only)
double tend = 50.0;

// Output directories (Povray only)
const std::string out_dir = "../DOUBLELANECHANGE_HMMWV";
const std::string pov_dir = out_dir + "/POVRAY";

// POV-Ray output
bool povray_output = true;

// Vehicle state output (forced to true if povray output enabled)
bool state_output = true;
int filter_window_size = 20;

// =============================================================================

int main(int argc, char* argv[]) {

  // ----------------------
  // Create the Bezier path
  // ----------------------

  ChBezierCurve* path = ChBezierCurve::read(vehicle::GetDataFile(path_file));

  // --------------------------
  // Create the various modules
  // --------------------------

  // Create the vehicle system
  WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), ChMaterialSurfaceBase::DEM);
  vehicle.Initialize(ChCoordsys<>(initLoc, initRot));
  //vehicle.GetSystem()->SetIterLCPmaxItersSpeed(1000);
  //vehicle.GetSystem()->SetIterLCPmaxItersStab(1000);
  vehicle.GetSystem()->SetLcpSolverType(ChSystem::LCP_ITERATIVE_MINRES);
  vehicle.GetSystem()->SetIterLCPwarmStarting(true);
  //vehicle.GetSystem()->SetTolForce(1e-2);
  chrono::ChLcpIterativeSolver* msolver_speed = (chrono::ChLcpIterativeSolver*) vehicle.GetSystem()->GetLcpSolverSpeed();
  msolver_speed->SetRecordViolation(true);

  // Create the ground
  //RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
  double mu = 0.9;
  double restitution = 0.01;
  double E = 2e7;
  double nu = 0.3;
  RigidTerrain terrain(vehicle.GetSystem());
  terrain.SetContactMaterial(mu, restitution, E, nu);
  terrain.SetColor(ChColor(0.8f, 0.8f, 0.5f));
  terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
  terrain.Initialize(terrainHeight, terrainLength, terrainWidth);
  
  // Create and initialize the powertrain system
  ShaftsPowertrain powertrain(vehicle::GetDataFile(shaftspowertrain_file));
  powertrain.Initialize(vehicle.GetChassis(), vehicle.GetDriveshaft());

  // Create and initialize the tires
  int num_axles = vehicle.GetNumberAxles();
  int num_wheels = 2 * num_axles;
  std::vector<std::shared_ptr<ChTire> > tires(num_wheels);

  for (int i = 0; i < num_wheels; i++) {
    tires[i] = std::make_shared<ChPacejkaTire>("FL", vehicle::GetDataFile(pactire_file));
    chrono::vehicle::VehicleSide side = LEFT;
    if (i % 2) side = RIGHT;
    tires[i]->Initialize(vehicle.GetWheelBody(i), side);
    std::dynamic_pointer_cast<ChPacejkaTire>(tires[i])->SetStepsize(step_size);
    //tires[i]->Initialize(vehicle.GetWheelBody(i)); 
  }

#ifdef USE_IRRLICHT

  ChVehicleIrrApp app(&vehicle, &powertrain, L"Vehicle Demo");

  app.SetSkyBox();
  app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
  app.SetChaseCamera(trackPoint, 6.0, 0.5);

  app.SetTimestep(step_size);

  /*
  bool do_shadows = false; // shadow map is experimental
  irr::scene::ILightSceneNode* mlight = 0;

  if (do_shadows) {
  mlight = application.AddLightWithShadow(
  irr::core::vector3df(10.f, 30.f, 60.f),
  irr::core::vector3df(0.f, 0.f, 0.f),
  150, 60, 80, 15, 512, irr::video::SColorf(1, 1, 1), false, false);
  } else {
  application.AddTypicalLights(
  irr::core::vector3df(30.f, -30.f, 100.f),
  irr::core::vector3df(30.f, 50.f, 100.f),
  250, 130);
  }

  if (do_shadows)
  application.AddShadowAll();
  */

  // Visualization of controller points (sentinel & target)
  irr::scene::IMeshSceneNode* ballS = app.GetSceneManager()->addSphereSceneNode(0.1f);
  irr::scene::IMeshSceneNode* ballT = app.GetSceneManager()->addSphereSceneNode(0.1f);
  ballS->getMaterial(0).EmissiveColor = irr::video::SColor(0, 255, 0, 0);
  ballT->getMaterial(0).EmissiveColor = irr::video::SColor(0, 0, 255, 0);

  //ChIrrGuiDriver driver(app);
  ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
    vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);

  // Set the time response for steering and throttle keyboard inputs.
  // NOTE: this is not exact, since we do not render quite at the specified FPS.
  double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
  double throttle_time = 1.0;  // time to go from 0 to +1
  double braking_time = 0.3;   // time to go from 0 to +1
  //driver.SetSteeringDelta(render_step_size / steering_time);
  //driver.SetThrottleDelta(render_step_size / throttle_time);
  //driver.SetBrakingDelta(render_step_size / braking_time);

  // Set file with driver input time series
  //driver.SetInputDataFile(vehicle::GetDataFile(driver_file));

  app.AssetBindAll();
  app.AssetUpdateAll();

#else

  //ChDataDriver driver(vehicle, vehicle::GetDataFile(driver_file));
  ChPathFollowerDriver driver(vehicle, vehicle::GetDataFile(steering_controller_file),
    vehicle::GetDataFile(speed_controller_file), path, "my_path", target_speed);

#endif

  state_output = state_output || povray_output;

  if (state_output) {
    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
      std::cout << "Error creating directory " << out_dir << std::endl;
      return 1;
    }
  }

  if (povray_output) {
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
      std::cout << "Error creating directory " << pov_dir << std::endl;
      return 1;
    }
    driver.ExportPathPovray(out_dir);
  }

  utils::CSV_writer csv("\t");
  csv.stream().setf(std::ios::scientific | std::ios::showpos);
  csv.stream().precision(6);

  utils::ChRunningAverage fwd_acc_GC_filter(filter_window_size);
  utils::ChRunningAverage lat_acc_GC_filter(filter_window_size);

  utils::ChRunningAverage fwd_acc_driver_filter(filter_window_size);
  utils::ChRunningAverage lat_acc_driver_filter(filter_window_size);

  // Driver location in vehicle local frame
  ChVector<> driver_pos = vehicle.GetLocalDriverCoordsys().pos;

  // ---------------
  // Simulation loop
  // ---------------

  // Inter-module communication data
  TireForces tire_forces(num_wheels);
  WheelStates wheel_states(num_wheels);
  double driveshaft_speed;
  double powertrain_torque;
  double throttle_input;
  double steering_input;
  double braking_input;

  // Number of simulation steps between two 3D view render frames
  int render_steps = (int)std::ceil(render_step_size / step_size);

  // Number of simulation steps between two output frames
  int output_steps = (int)std::ceil(output_step_size / step_size);

  // Initialize simulation frame counter and simulation time
  int step_number = 0;
  double time = 0;
  int render_frame = 0;
  double theta = 0;
  bool pinnedTireToGround = false;

#ifdef USE_IRRLICHT

  ChRealtimeStepTimer realtime_timer;


  while (app.GetDevice()->run()) {
    // Extract system state
    double time = vehicle.GetSystem()->GetChTime();
    ChVector<> vel_CG = vehicle.GetChassis()->GetPos_dt();
    ChVector<> acc_CG = vehicle.GetChassis()->GetPos_dtdt();
    acc_CG = vehicle.GetChassis()->GetCoord().TransformDirectionParentToLocal(acc_CG);
    ChVector<> acc_driver = vehicle.GetVehicleAcceleration(driver_pos);
    double fwd_acc_CG = fwd_acc_GC_filter.Add(acc_CG.x);
    double lat_acc_CG = lat_acc_GC_filter.Add(acc_CG.y);
    double fwd_acc_driver = fwd_acc_driver_filter.Add(acc_driver.x);
    double lat_acc_driver = lat_acc_driver_filter.Add(acc_driver.y);

    // Update sentinel and target location markers for the path-follower controller.
    // Note that we do this whether or not we are currently using the path-follower driver.
    const ChVector<>& pS = driver.GetSteeringController().GetSentinelLocation();
    const ChVector<>& pT = driver.GetSteeringController().GetTargetLocation();
    ballS->setPosition(irr::core::vector3df((irr::f32)pS.x, (irr::f32)pS.y, (irr::f32)pS.z));
    ballT->setPosition(irr::core::vector3df((irr::f32)pT.x, (irr::f32)pT.y, (irr::f32)pT.z));

    // Render scene
    if (step_number % render_steps == 0) {
      // Update the position of the shadow mapping so that it follows the car
      ////if (do_shadows) {
      ////  ChVector<> lightaim = vehicle.GetChassisPos();
      ////  ChVector<> lightpos = vehicle.GetChassisPos() + ChVector<>(10, 30, 60);
      ////  irr::core::vector3df mlightpos((irr::f32)lightpos.x, (irr::f32)lightpos.y, (irr::f32)lightpos.z);
      ////  irr::core::vector3df mlightaim((irr::f32)lightaim.x, (irr::f32)lightaim.y, (irr::f32)lightaim.z);
      ////  application.GetEffects()->getShadowLight(0).setPosition(mlightpos);
      ////  application.GetEffects()->getShadowLight(0).setTarget(mlightaim);
      ////  mlight->setPosition(mlightpos);
      ////}

      // Draw all scene elements
      app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
      app.DrawAll();
      app.EndScene();

      if (povray_output) {
        char filename[100];
        sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
        utils::WriteShapesPovray(vehicle.GetSystem(), filename);
      }

      if (state_output) {
        csv << time << steering_input << throttle_input << braking_input;
        csv << vehicle.GetVehicleSpeed();
        csv << acc_CG.x << fwd_acc_CG << acc_CG.y << lat_acc_CG;
        csv << acc_driver.x << fwd_acc_driver << acc_driver.y << lat_acc_driver;
        csv << acc_CG.z; // vertical acceleration
        csv << vel_CG.x << vel_CG.y << vel_CG.z;
        for (int i = 0; i < num_wheels; i++) {
          csv << tires[i]->GetTireForce().force.x << tires[i]->GetTireForce().force.y << tires[i]->GetTireForce().force.z;
        }
        csv << vehicle.GetChassis()->GetPos().x << vehicle.GetChassis()->GetPos().y << vehicle.GetChassis()->GetPos().z;
        csv << 180.0*theta / CH_C_PI; // ground angle
        csv << 180.0*(theta - vehicle.GetChassis()->GetA().Get_A_Rxyz().x) / CH_C_PI; // vehicle roll
        csv << 180.0*(vehicle.GetChassis()->GetA().Get_A_Rxyz().y) / CH_C_PI; // vehicle pitch
        csv << 180.0*(vehicle.GetChassis()->GetA().Get_A_Rxyz().z) / CH_C_PI; // vehicle yaw
        csv << std::endl;
      }

      render_frame++;
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();
    powertrain_torque = powertrain.GetOutputTorque();
    driveshaft_speed = vehicle.GetDriveshaftSpeed();
    for (int i = 0; i < num_wheels; i++) {
      tire_forces[i] = tires[i]->GetTireForce();
      wheel_states[i] = vehicle.GetWheelState(i);
    }

    // Update modules (process inputs from other modules)
    time = vehicle.GetSystem()->GetChTime();
    driver.Synchronize(time);
    powertrain.Synchronize(time, throttle_input, driveshaft_speed);
    vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
    terrain.Synchronize(time);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Synchronize(time, wheel_states[i], terrain);
    //app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);
    std::string msg = "Follower driver";
    app.Synchronize(msg, steering_input, throttle_input, braking_input);

    // Advance simulation for one timestep for all modules
    double step = realtime_timer.SuggestSimulationStep(step_size);
    driver.Advance(step);
    powertrain.Advance(step);
    vehicle.Advance(step);
    terrain.Advance(step);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Advance(step);
    app.Advance(step);

    // Increment frame number
    step_number++;
  }

  if (state_output) {
    std::stringstream fileNameStream;
    fileNameStream << "\\data_HMMWV_doubleLaneChange.dat";
    csv.write_to_file(out_dir + fileNameStream.str());
  }

#else

  //int render_frame = 0;

  if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << out_dir << std::endl;
    return 1;
  }
  if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
    std::cout << "Error creating directory " << pov_dir << std::endl;
    return 1;
  }

  char filename[100];

  while (time < tend) {
    if (step_number % render_steps == 0) {
      // Output render data
      sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
      utils::WriteShapesPovray(vehicle.GetSystem(), filename);
      std::cout << "Output frame:   " << render_frame << std::endl;
      std::cout << "Sim frame:      " << step_number << std::endl;
      std::cout << "Time:           " << time << std::endl;
      std::cout << "   throttle: " << driver.GetThrottle() << "   steering: " << driver.GetSteering()
        << "   braking:  " << driver.GetBraking() << std::endl;
      std::cout << std::endl;
      render_frame++;
    }

    // Collect output data from modules (for inter-module communication)
    throttle_input = driver.GetThrottle();
    steering_input = driver.GetSteering();
    braking_input = driver.GetBraking();
    powertrain_torque = powertrain.GetOutputTorque();
    driveshaft_speed = vehicle.GetDriveshaftSpeed();
    for (int i = 0; i < num_wheels; i++) {
      tire_forces[i] = tires[i]->GetTireForce();
      wheel_states[i] = vehicle.GetWheelState(i);
    }

    // Update modules (process inputs from other modules)
    time = vehicle.GetSystem()->GetChTime();
    driver.Synchronize(time);
    powertrain.Synchronize(time, throttle_input, driveshaft_speed);
    vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
    terrain.Synchronize(time);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Synchronize(time, wheel_states[i], terrain);

    // Advance simulation for one timestep for all modules
    driver.Advance(step_size);
    powertrain.Advance(step_size);
    vehicle.Advance(step_size);
    terrain.Advance(step_size);
    for (int i = 0; i < num_wheels; i++)
      tires[i]->Advance(step_size);

    // Increment frame number
    step_number++;
  }

#endif

  return 0;
}
