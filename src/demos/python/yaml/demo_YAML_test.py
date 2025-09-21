# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
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
# Simple demo for populating a Chrono system from a YAML model file.
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.parsers as parsers
import math

# -----------------------------------------------------------------------------

model_yaml_filename = 'yaml/models/slider_crank.yaml'
##model_yaml_filename = 'yaml/models/slider_crank_reduced.yaml'

sim_yaml_filename = 'yaml/simulations/basic_mbs.yaml'

# -----------------------------------------------------------------------------

# Create YAML parser object
parser = parsers.ChParserMbsYAML()
parser.SetVerbose(True)

# Load the YAML simulation file and create a Chrono system based on its content
parser.LoadSimulationFile(chrono.GetChronoDataFile(sim_yaml_filename))
sys = parser.CreateSystem()

# Load the YAML model and populate the Chrono system
parser.LoadModelFile(chrono.GetChronoDataFile(model_yaml_filename))
parser.Populate(sys)

# Extract information from parsed YAML files
model_name = parser.GetName()
time_end = parser.GetEndtime()
time_step = parser.GetTimestep()
real_time = parser.EnforceRealtime()
render = parser.Render()
render_fps = parser.GetRenderFPS()

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280,800)
vis.SetWindowTitle('YAML model - ' + model_name)
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddCamera(chrono.ChVector3d(2, -6, 0), chrono.ChVector3d(2, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.2, 0.2, 20, 20, chrono.ChCoordsysd(chrono.VNULL, chrono.Q_ROTATE_Y_TO_Z), chrono.ChColor(0.4, 0.4, 0.4))

# Simulation loop
rt_timer = chrono.ChRealtimeStepTimer()
time = 0.0
render_frame = 0

while time_end <= 0 or time < time_end: 
    if render and time >= render_frame / render_fps:
        ok = vis.Run()
        if not ok:
            break
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame = render_frame + 1

    sys.DoStepDynamics(time_step)
    if real_time:
        rt_timer.Spin(time_step)
    time = time + time_step
