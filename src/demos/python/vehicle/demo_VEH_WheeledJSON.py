# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import errno
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
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create the ground
    terrain = veh.RigidTerrain(vehicle.GetSystem(), rigidterrain_file)
    terrain.Initialize()

    # Create and initialize the powertrain system
    powertrain = veh.SimplePowertrain(simplepowertrain_file)
    vehicle.InitializePowertrain(powertrain)

    # Create and initialize the tires
    for axle in vehicle.GetAxles() :
        tireL = veh.RigidTire(rigidtire_file)
        vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
        tireR = veh.RigidTire(rigidtire_file)
        vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vehicle.SetVisualSystem(vis)
    vis.SetWindowTitle('HMMWV JSON specification')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(trackPoint, 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddTypicalLights()
    vis.AddSkyBox()

    driver = veh.ChIrrGuiDriver(vis)

    # Set the time response for steering and throttle keyboard inputs.
    # NOTE: this is not exact, since we do not render quite at the specified FPS.
    steering_time = 1.0;  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0;  # time to go from 0 to +1
    braking_time = 0.3;   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    driver.Initialize()

    # -----------------
    # Initialize output
    # -----------------

    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
           print("Error creating output directory " )

    # Generate JSON information with available output channels
    out_json = vehicle.ExportComponentList()
    print(out_json)
    vehicle.ExportComponentList(out_dir + "/component_list.json")

    # ---------------
    # Simulation loop
    # ---------------

    vehicle.EnableRealtime(True);
    while vis.Run() :

        # Render scene
        vis.BeginScene()
        vis.DrawAll()
        vis.EndScene()

        # Collect output data from modules (for inter-module communication)
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(driver.GetInputModeAsString(), driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# JSON file for vehicle model
vehicle_file = veh.GetDataFile('hmmwv/vehicle/HMMWV_Vehicle.json')

# JSON files for terrain
rigidterrain_file = veh.GetDataFile('terrain/RigidPlane.json')

# JSON file for powertrain (simple)
simplepowertrain_file = veh.GetDataFile('generic/powertrain/SimplePowertrain.json')

# JSON files tire models (rigid)
rigidtire_file = veh.GetDataFile('hmmwv/tire/HMMWV_RigidTire.json')

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

# Output directories
out_dir =  './WHEELED_JSON';

main()
