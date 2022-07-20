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
# Authors: Radu Serban
# =============================================================================
#
# Demonstration of vehicle over SCM deformable terrain
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left. All units SI.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import math as m

# =============================================================================

class MyDriver (veh.ChDriver):
	def __init__(self, vehicle, delay):
		veh.ChDriver.__init__(self, vehicle)
		self.delay = delay
	def Synchronize(self, time):
		eff_time = time - self.delay
		if (eff_time < 0):
		    return

		if (eff_time > 0.2):
			self.SetThrottle(0.7)
		else:
			self.SetThrottle(3.5 * eff_time)

		if (eff_time < 2):
			self.SetSteering(0.0)
		else:
			self.SetSteering(0.6 * m.sin(2.0 * m.pi * (eff_time - 2) / 6))

		self.SetBraking(0.0)

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    my_hmmwv = veh.HMMWV_Full()
    my_hmmwv.SetContactMethod(chrono.ChContactMethod_SMC)
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(chrono.ChVectorD(-5, -2, 0.6), chrono.ChQuaternionD(1, 0, 0, 0)))
    my_hmmwv.SetPowertrainType(veh.PowertrainModelType_SHAFTS)
    my_hmmwv.SetDriveType(veh.DrivelineTypeWV_AWD)
    my_hmmwv.SetTireType(veh.TireModelType_RIGID)
    my_hmmwv.Initialize()

    my_hmmwv.SetChassisVisualizationType(veh.VisualizationType_NONE)
    my_hmmwv.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    my_hmmwv.SetWheelVisualizationType(veh.VisualizationType_NONE)
    my_hmmwv.SetTireVisualizationType(veh.VisualizationType_MESH)

    # Create the (custom) driver
    driver = MyDriver(my_hmmwv.GetVehicle(), 0.5)
    driver.Initialize()

    # Create the SCM deformable terrain patch
    terrain = veh.SCMDeformableTerrain(my_hmmwv.GetSystem())
    terrain.SetSoilParameters(2e6,   # Bekker Kphi
                              0,     # Bekker Kc
                              1.1,   # Bekker n exponent
                              0,     # Mohr cohesive limit (Pa)
                              30,    # Mohr friction limit (degrees)
                              0.01,  # Janosi shear coefficient (m)
                              2e8,   # Elastic stiffness (Pa/m), before plastic yield
                              3e4    # Damping (Pa s/m), proportional to negative vertical speed (optional)
    )

    # Optionally, enable moving patch feature (single patch around vehicle chassis)
    terrain.AddMovingPatch(my_hmmwv.GetChassisBody(), chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(5, 3, 1))

    # Set plot type for SCM (false color plotting)
    terrain.SetPlotType(veh.SCMDeformableTerrain.PLOT_SINKAGE, 0, 0.1);

    # Initialize the SCM terrain, specifying the initial mesh grid
    terrain.Initialize(terrainLength, terrainWidth, delta);

    # Create the vehicle Irrlicht interface
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('HMMWV Deformable Soil Demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddTypicalLights()
    vis.AddSkyBox()
    vis.AttachVehicle(my_hmmwv.GetVehicle())

    # Simulation loop
    while vis.Run() :
        time = my_hmmwv.GetSystem().GetChTime()

        # End simulation
        if (time >= 4):
            break

        # Draw scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()


        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        my_hmmwv.Synchronize(time, driver_inputs, terrain)
        vis.Synchronize("", driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        my_hmmwv.Advance(step_size)
        vis.Advance(step_size)

    return 0
  

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# SCM patch dimensions
terrainHeight = 0
terrainLength = 16.0  # size in X direction
terrainWidth = 8.0    # size in Y direction

# SCM grid spacing
delta = 0.05

# Simulation step sizes
step_size = 2e-3;
tire_step_size = 1e-3;


main()
