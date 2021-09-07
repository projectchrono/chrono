import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import random

def main():
    #------------------
    # Create the system
    #------------------
    mphysicalSystem = chrono.ChSystemNSC()
    mphysicalSystem.Set_G_acc(chrono.ChVectorD(0,0,0))

    red = chrono.ChVisualMaterial()
    red.SetDiffuseColor(chrono.ChVectorF(1,0,0))
    red.SetSpecularColor(chrono.ChVectorF(1,1,1))

    green = chrono.ChVisualMaterial()
    green.SetDiffuseColor(chrono.ChVectorF(0,1,0))
    green.SetSpecularColor(chrono.ChVectorF(1,1,1))

    #------------------------------
    # add body for sensor to attach
    #------------------------------

    floor = chrono.ChBodyEasyBox(1000,20,1, 1000, True, False)
    floor.SetPos(chrono.ChVectorD(0,0,-1))
    floor.SetBodyFixed(True)
    mphysicalSystem.Add(floor)

    for i in range(10):
        x = random.uniform(0,30)
        y = 1
        z = 0
        box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, False)
        box.SetPos(chrono.ChVectorD(5+x, y, z))
        box.SetPos_dt(chrono.ChVectorD(-0.5, 0, 0))
        mphysicalSystem.Add(box)
        asset = floor.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(asset)
        visual_asset.material_list.append(red)
    
    for i in range(10):
        x = random.uniform(0,30)
        y = -1
        z = 0
        box = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 1000, True, False)
        box.SetPos(chrono.ChVectorD(10-x, y, z))
        box.SetPos_dt(chrono.ChVectorD(0.5, 0, 0))
        mphysicalSystem.Add(box)
        asset = floor.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(asset)
        visual_asset.material_list.append(red)

    # -----------------------
    # Create a sensor manager
    # -----------------------
    manager = sens.ChSensorManager(mphysicalSystem)
    # ------------------------------------------------
    # Create a radar and add it to the sensor manager
    # ------------------------------------------------
    offset_pose = chrono.ChFrameD(
        chrono.ChVectorD(0, 0, 1), chrono.Q_from_AngZ(0))
    radar = sens.ChRadarSensor(
        floor,              # body radar is attached to
        update_rate,            # scanning rate in Hz
        offset_pose,            # offset pose
        horizontal_samples,     # number of horizontal samples
        vertical_samples,       # number of vertical channels
        horizontal_fov,         # horizontal field of view
        max_vert_angle,         # vertical field of view
        min_vert_angle,
        100.0,  # max radar range
    )
    radar.PushFilter(sens.ChFilterRadarProcess())
    radar.PushFilter(sens.ChFilterRadarVisualizeCluster(960, 1080, 2))

    manager.AddSensor(radar)
    # ---------------
    # Simulate system
    # ---------------
    orbit_radius = 5
    orbit_rate = 0.2
    ch_time = 0.0

    render_time = 0

    t1 = time.time()

    while (ch_time < end_time):

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Perform step of dynamics
        mphysicalSystem.DoStepDynamics(step_size)

        # Get the current time of the simulation
        ch_time = mphysicalSystem.GetChTime()

    print("Sim time:", end_time, "Wall time:", time.time()-t1)

# -----------------
# radar parameters
# -----------------


# Update rate in Hz
update_rate = 5.0

# Number of horizontal and vertical samples
horizontal_samples = 100
vertical_samples = 100

# Horizontal and vertical field of view (radians)
horizontal_fov =  chrono.CH_C_PI /9 # 20 degrees
max_vert_angle = chrono.CH_C_PI / 15
min_vert_angle = -chrono.CH_C_PI / 15

# camera to have same view as radar
aspect_ratio = horizontal_fov / (max_vert_angle - min_vert_angle)
width = 960
height = width / aspect_ratio

# Lag time
lag = 0

# Collection window for the radar
collection_time = 1. / update_rate  # typically 1/update rate

# ---------------------
# Simulation parameters
# ---------------------

# Simulation step size
step_size = 1e-3

# Simulation end time
end_time = 100.0


main()
