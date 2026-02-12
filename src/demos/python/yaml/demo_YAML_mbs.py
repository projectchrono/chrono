# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demo for using the Chrono YAML parser in python
#
# =============================================================================

import pychrono as chrono
import pychrono.parsers as parsers
import pychrono.vsg3d as vsg
import errno
import os

def main():
    # Create the YAML parser
    parser = parsers.ChParserMbsYAML(model_yaml_filename, sim_yaml_filename, True)

    # Create the Chrono system and populate it with the model    
    sys = parser.CreateSystem()
    parser.Populate(sys)

    # Extract information from parsed YAML files
    model_name = parser.GetName()
    time_end = parser.GetEndtime()
    time_step = parser.GetTimestep()
    real_time = parser.EnforceRealtime()
    render = parser.Render()
    render_fps = parser.GetRenderFPS()
    camera_vertical = parser.GetCameraVerticalDir()
    camera_location = parser.GetCameraLocation()
    camera_target = parser.GetCameraTarget()
    enable_shadows = parser.EnableShadows()
    output = parser.Output()
    output_fps = parser.GetOutputFPS()

    # Create the VSG visualization system
    if render:
        vis = vsg.ChVisualSystemVSG()
        vis.AttachSystem(sys)
        vis.SetWindowSize(chrono.ChVector2i(1200, 800))
        vis.SetWindowPosition(chrono.ChVector2i(100, 300))
        vis.SetWindowTitle("YAML model - " + model_name)
        vis.SetCameraVertical(camera_vertical)
        vis.AddCamera(camera_location, camera_target)
        vis.SetCameraAngleDeg(40)
        vis.SetLightIntensity(1.0)
        vis.SetLightDirection(-chrono.CH_PI_4, chrono.CH_PI_4)
        vis.EnableShadows(enable_shadows)
        vis.ToggleAbsFrameVisibility()
        vis.SetAbsFrameScale(2.0)
        vis.Initialize()

    # Create output directory
    if output:
        out_dir = chrono.GetChronoOutputPath() + "YAML_MBS/"
        if not os.path.exists(out_dir):
            try:
                os.mkdir(out_dir)
            except OSError as exc:
                if exc.errno != errno.EEXIST:
                    print("Error creating output directory ")
        out_dir = out_dir + "/" + model_name
        if not os.path.exists(out_dir):
            try:
                os.mkdir(out_dir)
            except OSError as exc:
                if exc.errno != errno.EEXIST:
                    print("Error creating output directory ")
        parser.SetOutputDir(out_dir)

    # Simulation loop
    rt_timer = chrono.ChRealtimeStepTimer()
    time = 0
    render_frame = 0
    output_frame = 0

    while True:
        if render:
            if not vis.Run():
                break
            if time >= render_frame / render_fps:
                vis.BeginScene()
                vis.Render()
                vis.EndScene()
                render_frame += 1

        if output:
            if time >= output_frame / output_fps:
                parser.SaveOutput(sys, output_frame)
                output_frame += 1

        sys.DoStepDynamics(time_step)
        if real_time:
            rt_timer.Spin(time_step)

        time += time_step

# =============================================================================

# Set YAML model and simulation files
model_yaml_filename = chrono.GetChronoDataFile("yaml/mbs/slider_crank.yaml")
sim_yaml_filename = chrono.GetChronoDataFile("yaml/mbs/simulation_mbs.yaml")

# Set output root directory
chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

main()
