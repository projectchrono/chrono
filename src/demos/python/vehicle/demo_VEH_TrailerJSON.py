import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import os
import math as m


# =============================================================================

def main() : 
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # --------------------------
    # Create the various modules
    # --------------------------

    # Create the vehicle system
    vehicle = veh.WheeledVehicle(vehicle_file, chrono.ChContactMethod_NSC)
    vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))
    #vehicle.GetChassis().SetFixed(True)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)

    # Create and initialize the vehicle tires
    for axle in vehicle.GetAxles() :
        tireL = veh.TMeasyTire(vehicle_tire_file)
        vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
        tireR = veh.TMeasyTire(vehicle_tire_file)
        vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

    # Create and initialize the powertrain system
    powertrain = veh.SimpleMapPowertrain(vehicle_powertrain_file)
    vehicle.InitializePowertrain(powertrain)

    # Create and initialize the trailer
    trailer = veh.WheeledTrailer(vehicle.GetSystem(), trailer_file)
    trailer.Initialize(vehicle.GetChassis())
    trailer.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    trailer.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    trailer.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create abd initialize the trailer tires
    for axle in trailer.GetAxles() :
        tireL = veh.TMeasyTire(trailer_tire_file)
        trailer.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_PRIMITIVES)
        tireR = veh.TMeasyTire(trailer_tire_file)
        trailer.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_PRIMITIVES)

    # Create the ground
    terrain = veh.RigidTerrain(vehicle.GetSystem(), rigidterrain_file)

    app = veh.ChVehicleIrrApp(vehicle, 'Sedan+Trailer (JSON specification)')
    app.AddTypicalLights()
    app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.SetChaseCamera(trackPoint, 6.0, 0.5)
    app.SetTimestep(step_size)
    app.AssetBindAll()
    app.AssetUpdateAll()

    driver = veh.ChIrrGuiDriver(app)

    # Set the time response for steering and throttle keyboard inputs.
    # NOTE: this is not exact, since we do not render quite at the specified FPS.
    steering_time = 1.0;  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0;  # time to go from 0 to +1
    braking_time = 0.3;   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # ---------------
    # Simulation loop
    # ---------------

    realtime_timer = chrono.ChRealtimeStepTimer()
    while (app.GetDevice().run()) :

        # Render scene
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

        # Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        trailer.Synchronize(time, driver_inputs.m_braking, terrain)
        terrain.Synchronize(time)
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        trailer.Advance(step_size)
        terrain.Advance(step_size)
        app.Advance(step_size)

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# JSON files for vehicle model
vehicle_file = veh.GetDataFile('sedan/vehicle/Sedan_Vehicle.json')
vehicle_tire_file = veh.GetDataFile('sedan/tire/Sedan_TMeasyTire.json')
vehicle_powertrain_file = veh.GetDataFile('sedan/powertrain/Sedan_SimpleMapPowertrain.json')

# JSON files for trailer model
trailer_file = veh.GetDataFile('ultra_tow/UT_Trailer.json')
trailer_tire_file = veh.GetDataFile('ultra_tow/UT_TMeasyTire.json')

# JSON files for terrain
rigidterrain_file = veh.GetDataFile('terrain/RigidPlane.json')

# Initial vehicle position
initLoc = chrono.ChVectorD(0, 0, 0.5)

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


main()
