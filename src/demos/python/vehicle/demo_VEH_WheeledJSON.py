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
import errno
import os

# =============================================================================

def main() : 
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    # Create the vehicle system
    vehicle = veh.WheeledVehicle(vehicle_file, chrono.ChContactMethod_NSC)
    vehicle.Initialize(chrono.ChCoordsysd(initLoc, initRot))
    #vehicle.GetChassis().SetFixed(True)
    vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetChassisRearVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create and initialize the powertrain system
    engine = veh.ReadEngineJSON(engine_file)
    transmission = veh.ReadTransmissionJSON(transmission_file)
    powertrain = veh.ChPowertrainAssembly(engine, transmission)
    vehicle.InitializePowertrain(powertrain)

    # Create and initialize the tires
    for axle in vehicle.GetAxles() :
        for wheel in axle.GetWheels() :
            tire = veh.ReadTireJSON(tire_file)
            vehicle.InitializeTire(tire, wheel, veh.VisualizationType_MESH)

    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem(), rigidterrain_file)
    terrain.Initialize()

    # Create Irrilicht visualization
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV JSON specification')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    # Create the interactive driver
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # Initialize output
    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
           print("Error creating output directory " )

    # Generate JSON information with available output channels
    out_json = vehicle.ExportComponentList()
    print(out_json)
    vehicle.ExportComponentList(out_dir + "/component_list.json")

    # Simulation loop
    vehicle.EnableRealtime(True)

    while vis.Run() :
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

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

# Terain JSON specification file
rigidterrain_file = veh.GetDataFile('terrain/RigidPlane.json')

# HMMWV specification files (vehicle, powertrain, and tire models)
vehicle_file = veh.GetDataFile('hmmwv/vehicle/HMMWV_Vehicle.json')
engine_file = veh.GetDataFile('hmmwv/powertrain/HMMWV_EngineShafts.json')
transmission_file = veh.GetDataFile('hmmwv/powertrain/HMMWV_AutomaticTransmissionShafts.json')
tire_file = veh.GetDataFile('hmmwv/tire/HMMWV_Pac02Tire.json')

# ACV specification files (vehicle, powertrain, and tire models)
#vehicle_file = veh.GetDataFile('articulated_chassis/ACV_Vehicle.json')
#powertrain_file = veh.GetDataFile('articulated_chassis/ACV_SimplePowertrain.json')
#tire_file = veh.GetDataFile('articulated_chassis/ACV_RigidTire.json')

# Initial vehicle position
initLoc = chrono.ChVector3d(0, 0, 0.5)

# Initial vehicle orientation
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50  # FPS = 50

# Output directories
out_dir =  './WHEELED_JSON';

main()
