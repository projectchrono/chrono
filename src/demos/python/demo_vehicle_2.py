import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math as m


# =============================================================================

def main() : 
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------------------
    # Create the various modules
    # --------------------------

    # Create the vehicle system
    vehicle = veh.WheeledVehicle(vehicle_file ,chrono.ChMaterialSurface.NSC)
    vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))
    #vehicle.GetChassis().SetFixed(True)
    vehicle.SetStepsize(step_size)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create the ground
    terrain = veh.RigidTerrain(vehicle.GetSystem(), rigidterrain_file)

    # Create and initialize the powertrain system
    powertrain = veh.SimplePowertrain(simplepowertrain_file)
    powertrain.Initialize(vehicle.GetChassisBody(), vehicle.GetDriveshaft())

    #// Create and initialize the tires
    num_axles = vehicle.GetNumberAxles()
    num_wheels = 2 * num_axles
    tires = [ veh.RigidTire(rigidtire_file) for i in range(num_wheels)]

    for i, t in enumerate(tires):
        #t = std::make_shared<vehicle::RigidTire>(vehicle::GetDataFile(rigidtire_file));
       s = [veh.LEFT, veh.RIGHT]
       t.Initialize(vehicle.GetWheelBody(veh.WheelID(i)), s[i % 2])
       t.SetVisualizationType(veh.VisualizationType_MESH)


    app = veh.ChVehicleIrrApp (vehicle, powertrain)

    app.SetSkyBox()
    app.AddTypicalLights(chronoirr.vector3df(30, -30, 100), chronoirr.vector3df(30, 50, 100), 250, 130)
    app.SetChaseCamera(trackPoint, 6.0, 0.5)

    app.SetTimestep(step_size)

    app.AssetBindAll()
    app.AssetUpdateAll()

    """
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
    """

    driver = veh.ChIrrGuiDriver(app)

    # Set the time response for steering and throttle keyboard inputs.
    # NOTE: this is not exact, since we do not render quite at the specified FPS.
    steering_time = 1.0;  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0;  # time to go from 0 to +1
    braking_time = 0.3;   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    # Set file with driver input time series
    driver.SetInputDataFile(driver_file)
    """
       #else
       
           ChDataDriver driver(vehicle, vehicle::GetDataFile(driver_file));
       
       #endif
    """
    driver.Initialize()

    # -----------------
    # Initialize output
    # -----------------

    try:
           os.mkdir(out_dir)
    except:
           print("Error creating directory " )
    
    """if (povray_output):
        try: 
            os.mkdir(pov_dir)
        except:
            print("Error creating POV directory ")
        terrain.ExportMeshPovray(out_dir)"""

    # Generate JSON information with available output channels
    out_json = vehicle.ExportComponentList()
    print(out_json)
    vehicle.ExportComponentList(out_dir + "/component_list.json")

    # ---------------
    # Simulation loop
    # ---------------

    # Inter-module communication data
    tire_forces = veh.TerrainForces(num_wheels)
    wheel_states = veh.WheelStates(num_wheels)

    # Initialize simulation frame counter and simulation time
    step_number = 0
    time = 0

#ifdef USE_IRRLICHT

    realtime_timer = chrono.ChRealtimeStepTimer()

    while (app.GetDevice().run()) :

        # Render scene
        app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
        app.DrawAll()

        # Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle()
        steering_input = driver.GetSteering()
        braking_input = driver.GetBraking()
        powertrain_torque = powertrain.GetOutputTorque()
        driveshaft_speed = vehicle.GetDriveshaftSpeed()
        for i in range (num_wheels) :
            tire_forces[i] = tires[i].GetTireForce()
            wheel_states[i] = vehicle.GetWheelState(veh.WheelID(i))

        # Update modules (process inputs from other modules)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        powertrain.Synchronize(time, throttle_input, driveshaft_speed)
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces)
        terrain.Synchronize(time)
        for i in range (num_wheels):
            tires[i].Synchronize(time, wheel_states[i], terrain)
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input)

        # Advance simulation for one timestep for all modules
        step = realtime_timer.SuggestSimulationStep(step_size)
        driver.Advance(step)
        powertrain.Advance(step)
        vehicle.Advance(step)
        terrain.Advance(step)
        for i in range(num_wheels) :
            tires[i].Advance(step)
        app.Advance(step)

        # Increment frame number
        step_number += 1

        app.EndScene()
"""
#else

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    int render_frame = 0;
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
"""



# =============================================================================
povray_output = False
# JSON file for vehicle model
veh.SetDataPath('../../../../Library/data/vehicle/')
CHRONO_DATA_DIR = "../../../../Library/data/"
chrono.SetChronoDataPath(CHRONO_DATA_DIR);
vehicle_file = veh.GetDataPath() +"hmmwv/vehicle/HMMWV_Vehicle.json"

# JSON files for terrain
rigidterrain_file = veh.GetDataPath() +"terrain/RigidPlane.json"

# JSON file for powertrain (simple)
simplepowertrain_file = veh.GetDataPath() + "generic/powertrain/SimplePowertrain.json"

# JSON files tire models (rigid)
rigidtire_file = veh.GetDataPath() +"hmmwv/tire/HMMWV_RigidTire.json"

# Driver input file (if not using Irrlicht)
driver_file = ("generic/driver/Sample_Maneuver.txt");

# Initial vehicle position
initLoc = chrono.ChVectorD(0, 0, 1.6)

# Initial vehicle orientation
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Rigid terrain dimensions
terrainHeight = 0
terrainLength = 300.0  # size in X direction
terrainWidth = 200.0   # size in Y direction

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Point on chassis tracked by the camera (Irrlicht only)
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Simulation length (Povray only)
tend = 20.0

# Output directories (Povray only)
out_dir =  "./WHEELED_JSON";
#const std::string pov_dir = out_dir + "/POVRAY";

main()