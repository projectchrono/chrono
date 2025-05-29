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
import math

# -----------------------------------------------------------------------------

model_yaml_filename = 'yaml/slider_crank.yaml'
##model_yaml_filename = 'yaml/slider_crank_reduced.yaml'

time_step = 1e-4
render_fps = 120

# Create the system
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
chrono.ChCollisionInfo.SetDefaultEffectiveCurvatureRadius(0.2)

# Create YAML parser object and load model file
parser = chrono.ChYamlParser()
parser.SetVerbose(True)
parser.Load(chrono.GetChronoDataFile(model_yaml_filename))
model_name = parser.GetName()

# Populate Chrono system with YAML model
instance1 = parser.Populate(sys)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280,800)
vis.SetWindowTitle('YAML model')
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddCamera(chrono.ChVector3d(2, -6, 0), chrono.ChVector3d(2, 0, 0))
vis.AddTypicalLights()
vis.AddGrid(0.2, 0.2, 20, 20, chrono.ChCoordsysd(chrono.VNULL, chrono.Q_ROTATE_Y_TO_Z), chrono.ChColor(0.4, 0.4, 0.4))

# Simulation loop
rt_timer = chrono.ChRealtimeStepTimer()
time = 0.0
render_frame = 0

while vis.Run(): 
    if time >= render_frame / render_fps:
        vis.BeginScene()
        vis.Render()
        vis.EndScene()
        render_frame = render_frame + 1

    sys.DoStepDynamics(time_step)
    rt_timer.Spin(time_step)
    time = time + time_step

