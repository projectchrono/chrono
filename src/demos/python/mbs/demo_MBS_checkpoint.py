# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
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
# Writing and reading checkpoint files
#
# =============================================================================

import pychrono.core as chrono
import pychrono.vsg3d as vsg
import os

# -----------------------------------------------------------------------------

def ConstructModel(sys, id):
    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    crank = chrono.ChBody()
    crank.SetPos(chrono.ChVector3d(1, 0, 0))
    crank.SetMass(2)
    crank_shape = chrono.ChVisualShapeCylinder(0.03, 2)
    if id==1:
        crank_shape.SetColor(chrono.ChColor(0, 1, 0))
    else:
        crank_shape.SetColor(chrono.ChColor(0, 0, 1))
    crank.AddVisualShape(crank_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
    sys.AddBody(crank)

    rod = chrono.ChBody()
    rod.SetPos(chrono.ChVector3d(4, 0, 0))
    rod.SetMass(3)
    rod_shape = chrono.ChVisualShapeCylinder(0.015, 4)
    if id==1:
        rod_shape.SetColor(chrono.ChColor(0, 1, 0))
    else:
        rod_shape.SetColor(chrono.ChColor(0, 0, 1))
    rod.AddVisualShape(rod_shape, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_PI_2)))
    sys.AddBody(rod)

    revolute = chrono.ChLinkLockRevolute()
    revolute.Initialize(crank, rod, chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))
    sys.AddLink(revolute)

    point_line = chrono.ChLinkLockPointLine()
    point_line.Initialize(rod, ground, chrono.ChFramed(chrono.ChVector3d(6, 0, 0)))
    sys.AddLink(point_line)

    motor = chrono.ChLinkMotorRotationSpeed()
    motor.Initialize(ground, crank, chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
    sys.AddLink(motor)
    my_speed_function = chrono.ChFunctionConst(chrono.CH_PI)
    motor.SetSpeedFunction(my_speed_function)

    print("Done sys ", id)

# -----------------------------------------------------------------------------

def main():

    # Set output root directory
    chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

    # Create output directory
    out_dir = chrono.GetChronoOutputPath() + "DEMO_CHECKPOINT/"
    if not os.path.exists(out_dir):
        try:
            os.mkdir(out_dir)
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                print("Error creating output directory ")

    # Construct systems
    print(out_dir)
    print("Start")

    sys1 = chrono.ChSystemNSC()
    sys2 = chrono.ChSystemNSC()
    sys1.SetGravityY()
    sys2.SetGravityY()
    ConstructModel(sys1, 1)
    ConstructModel(sys2, 2)

    # Create the run-time visualization system
    vis = vsg.ChVisualSystemVSG()
    vis.AttachSystem(sys1)
    vis.AttachSystem(sys2)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Y)
    vis.SetWindowSize(chrono.ChVector2i(1280, 800))
    vis.SetWindowPosition(chrono.ChVector2i(100, 100))
    vis.SetWindowTitle("Slider-crank checkpointing")
    vis.AddCamera(chrono.ChVector3d(2, 0, 6), chrono.ChVector3d(2, 0, 0))
    vis.SetCameraAngleDeg(40)
    vis.SetLightIntensity(1.0)
    vis.SetLightDirection(chrono.CH_PI_2, chrono.CH_PI_4)
    vis.Initialize()

    # Checkpoint setup
    cp_filename = out_dir + "/checkpoint.txt"
    cp_time = 1.5
    cp_created = False

    print("Checkpint file: ", cp_filename)

    # Simulation loop
    rt_timer = chrono.ChRealtimeStepTimer()
    h = 1e-3
    t = 0

    while True:
        if not vis.Run():
            break

        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        if t < cp_time:
            # Simulate 1st system
            sys1.DoStepDynamics(h)
        else:
            # Checkpoint 1st system and initialize 2nd system
            if not cp_created:
                cp = chrono.ChCheckpointASCII(chrono.ChCheckpoint.Type_SYSTEM)
                cp.WriteState(sys1)
                cp.WriteFile(cp_filename)

                cp = chrono.ChCheckpointASCII(chrono.ChCheckpoint.Type_SYSTEM)
                cp.OpenFile(cp_filename)
                cp.ReadState(sys2)
               
                cp_created = True
           
            # Simulate 2nd system
            sys2.DoStepDynamics(h)

            # Force time of sys1 (for display purposes)
            sys1.SetChTime(sys2.GetChTime())

        t = t + h
        rt_timer.Spin(h)

# -----------------------------------------------------------------------------

if __name__ == "__main__":
    main()
