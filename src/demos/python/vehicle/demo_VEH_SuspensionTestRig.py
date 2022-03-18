# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2022 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demonstration program for a suspension test rig.
#
# Driver inputs for a suspension test rig include left/right post displacements
# and steering input (the latter being ignored if the tested suspension is not
# attached to a steering mechanism).  These driver inputs can be obtained from
# an interactive driver system (of type ChIrrGuiDriverSTR) or from a data file
# (using a driver system of type ChDataDriverSTR).
#
# See the description of ChSuspensionTestRig::PlotOutput for details on data
# collected (if output is enabled).
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

    # Create the rig from a JSON specification file
    rig = veh.ChSuspensionTestRigPushrod(str_file)

    # Create and initialize the tires
    # (not needed if the vehicle's suspension JSON specification files include tire data)
    
    for ia in test_axles:
        axle = rig.GetVehicle().GetAxle(ia)
        for wheel in axle.GetWheels():
            tire = veh.ReadTireJSON(tire_file)
            rig.GetVehicle().InitializeTire(tire, wheel, veh.VisualizationType_NONE)

    # Optional rig settings
    rig.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    rig.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    rig.SetSubchassisVisualizationType(veh.VisualizationType_PRIMITIVES)
    rig.SetWheelVisualizationType(veh.VisualizationType_NONE);
    rig.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the vehicle Irrlicht application
    cam_loc = (rig.GetSpindlePos(0, veh.LEFT) + rig.GetSpindlePos(0, veh.RIGHT)) * 0.5
    app = veh.ChVehicleIrrApp(rig.GetVehicle(), 'Suspension Test Rig')
    app.AddTypicalLights()
    app.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    app.SetChaseCamera(cam_loc, 4.0, 0.5)
    app.SetTimestep(step_size)

    # Create and attach an STR driver
    driver = veh.ChDataDriverSTR(driver_file)
    rig.SetDriver(driver)

    # Initialize suspension test rig
    rig.Initialize()
    app.AssetBindAll()
    app.AssetUpdateAll()

    # Set output
    try:
        os.mkdir(out_dir)
    except OSError as exc:
        if exc.errno != errno.EEXIST:
           print("Error creating output directory " )
    
    if output:
        rig.SetOutput(veh.ChVehicleOutput.ASCII, out_dir, 'output', out_step_size)
    if plot:
        rig.SetPlotOutput(out_step_size)

    # Simulation loop
    while (app.GetDevice().run()):

        # Render scene
        app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
        app.DrawAll()
        app.EndScene()

        # Advance simulation of the rig 
        rig.Advance(step_size)

        # Update visualization app
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = rig.GetSteeringInput()
        driver_inputs.m_throttle = 0.0
        driver_inputs.m_braking = 0.0
        app.Synchronize(rig.GetDriverMessage(), driver_inputs)
        app.Advance(step_size)

        if rig.DriverEnded():
            break

    rig.PlotOutput(out_dir, 'output_plot')

# =============================================================================

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# JSON file for suspension test rig
str_file = veh.GetDataFile('mtv/suspensionTest/MTV_ST_rear.json')

# JSON file for tire
tire_file = veh.GetDataFile('mtv/tire/FMTV_TMeasyTire.json')

# Driver data file
driver_file = veh.GetDataFile('mtv/suspensionTest/ST_inputs.dat')

# Vehicle axles included in test rig
test_axles = [1, 2]

# Simulation step size
step_size = 1e-3

# Output collection
output = True
plot = True
out_dir =  './SUSPENSION_TEST_RIG'
out_step_size = 1e-2

main()
