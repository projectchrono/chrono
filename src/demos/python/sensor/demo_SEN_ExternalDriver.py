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
# Demonstration of a steering path-follower and cruise control PID controlers. 
#
# The vehicle reference frame has Z up, X towards the front of the vehicle, and
# Y pointing to the left.
#
# =============================================================================

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as irr
import pychrono.sensor as sens
import numpy as np

#// =============================================================================

class ChSystem_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, system: chrono.ChSystem):
        super().__init__("ChSystem", id)

        self.system = system

    def Serialize(self, writer):
        writer.Key("time") << self.system.GetChTime()

class ChCameraSensor_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, cam: sens.ChCameraSensor):
        super().__init__("ChCameraSensor", id)

        self.cam = cam

    def Serialize(self, writer):
        rgba8_buffer = self.cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            shape = rgba8_data.shape
            writer.Key("width") << shape[1]
            writer.Key("height") << shape[0] 
            writer.Key("size") << shape[2]
            writer.Key("encoding") << "rgba8"
            writer.Key("image").PointerAsString(rgba8_data.ctypes.data, int(np.prod(shape)))

    def HasData(self) -> bool:
        rgba8_buffer = self.cam.GetMostRecentRGBA8Buffer()
        return rgba8_buffer.HasData()

class ChVehicle_DataGeneratorFunctor(veh.ChExternalDriver_DataGeneratorFunctor):
    def __init__(self, id: str, vehicle: veh.ChVehicle):
        super().__init__("ChVehicle", id)

        self.vehicle = vehicle

    def Serialize(self, writer):
        body = self.vehicle.GetChassisBody()

        writer.Key("pos") << body.GetPos()
        writer.Key("rot") << body.GetRot()
        writer.Key("lin_vel") << body.GetPos_dt()
        writer.Key("ang_vel") << body.GetWvel_loc()
        writer.Key("lin_acc") << body.GetPos_dtdt()
        writer.Key("ang_acc") << body.GetWacc_loc()

class ChDriverInputs_DataParserFunctor(veh.ChExternalDriver_DataParserFunctor):
    def __init__(self, driver: veh.ChDriver):
        super().__init__("ChDriverInputs")

        self.driver = driver

    def Deserialize(self, reader):
        steering = throttle = braking = 0.0
        reader >> steering >> throttle >> braking

        reader.SetThrottle(throttle)
        reader.SetSteering(steering)
        reader.SetBraking(braking)


#// =============================================================================

def main():
    #print("Copyright (c) 2017 projectchrono.org\nChrono version: ", CHRONO_VERSION , "\n\n")

    #  Create the HMMWV vehicle, set parameters, and initialize
    my_hmmwv = veh.HMMWV_Full()
    my_hmmwv.SetContactMethod(contact_method)
    my_hmmwv.SetChassisFixed(False) 
    my_hmmwv.SetInitPosition(chrono.ChCoordsysD(initLoc, chrono.ChQuaternionD(1, 0, 0, 0)))
    my_hmmwv.SetPowertrainType(powertrain_model)
    my_hmmwv.SetDriveType(drive_type)
    my_hmmwv.SetSteeringType(steering_type)
    my_hmmwv.SetTireType(tire_model)
    my_hmmwv.SetTireStepSize(tire_step_size)
    my_hmmwv.Initialize()

    my_hmmwv.SetChassisVisualizationType(chassis_vis_type)
    my_hmmwv.SetSuspensionVisualizationType(suspension_vis_type)
    my_hmmwv.SetSteeringVisualizationType(steering_vis_type)
    my_hmmwv.SetWheelVisualizationType(wheel_vis_type)
    my_hmmwv.SetTireVisualizationType(tire_vis_type)

    # Create the terrain

    terrain = veh.RigidTerrain(my_hmmwv.GetSystem())
    if (contact_method == chrono.ChContactMethod_NSC):
        patch_mat = chrono.ChMaterialSurfaceNSC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
    elif (contact_method == chrono.ChContactMethod_SMC):
        patch_mat = chrono.ChMaterialSurfaceSMC()
        patch_mat.SetFriction(0.9)
        patch_mat.SetRestitution(0.01)
        patch_mat.SetYoungModulus(2e7)
    patch = terrain.AddPatch(patch_mat, 
                             chrono.CSYSNORM, 
                             300, 50)
    patch.SetTexture(veh.GetDataFile("terrain/textures/tile4.jpg"), 200, 200)
    patch.SetColor(chrono.ChColor(0.8, 0.8, 0.5))
    terrain.Initialize()

    # Create the camera

    manager = sens.ChSensorManager(my_hmmwv.GetSystem())
    manager.scene.AddPointLight(chrono.ChVectorF(0, 0, 100), chrono.ChVectorF(2, 2, 2), 5000)

    cam_update_rate = 30
    offset_pose = chrono.ChFrameD(chrono.ChVectorD(-5, 0, 2))
    image_width = 280
    image_height = 120
    fov = 1.408
    cam = sens.ChCameraSensor(
        my_hmmwv.GetChassisBody(),              # body camera is attached to
        cam_update_rate,            # update rate in Hz
        offset_pose,            # offset pose
        image_width,            # image width
        image_height,           # image height
        fov                    # camera's horizontal field of view
    )
    cam.PushFilter(sens.ChFilterRGBA8Access())
    manager.AddSensor(cam)

    # Create the external driver
    driver = veh.ChExternalDriver(my_hmmwv.GetVehicle(), 50000)

    system_generator = ChSystem_DataGeneratorFunctor("~/output/time", my_hmmwv.GetSystem())
    driver.AddDataGenerator(system_generator)

    veh_generator = ChVehicle_DataGeneratorFunctor("~/output/vehicle", my_hmmwv.GetVehicle())
    driver.AddDataGenerator(veh_generator, 10)

    cam_generator = ChCameraSensor_DataGeneratorFunctor("~/output/camera/front_facing_camera", cam)
    driver.AddDataGenerator(cam_generator, cam_update_rate)

    inputs_parser = ChDriverInputs_DataParserFunctor(driver)
    driver.AddDataParser(inputs_parser)

    # Create the vehicle Irrlicht interface
    if USE_IRRLICHT:
        app = veh.ChWheeledVehicleIrrApp(my_hmmwv.GetVehicle(), 'HMMWV', irr.dimension2du(1000,800))
        app.SetSkyBox()
        app.AddTypicalLights(irr.vector3df(-60, -30, 100), irr.vector3df(60, 30, 100), 250, 130)
        app.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
        app.SetChaseCamera(chrono.ChVectorD(0.0, 0.0, 1.75), 6.0, 0.5)
        app.SetTimestep(step_size)
        app.AssetBindAll()
        app.AssetUpdateAll()

    # Simulation loop
    realtime_timer = chrono.ChRealtimeStepTimer()
    while True:
        time = my_hmmwv.GetSystem().GetChTime()

        # End simulation
        if (USE_IRRLICHT and not app.GetDevice().run()) or time >= t_end:
            break

        # Draw scene
        if USE_IRRLICHT:
            app.BeginScene(True, True, irr.SColor(255, 140, 161, 192))
            app.DrawAll()
            app.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        driver.Synchronize(time)
        terrain.Synchronize(time)
        my_hmmwv.Synchronize(time, driver_inputs, terrain)
        if USE_IRRLICHT:
            app.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        terrain.Advance(step_size)
        my_hmmwv.Advance(step_size)
        if USE_IRRLICHT:
            app.Advance(step_size)

        # Update sensor manager
        # Will render/save/filter automatically
        manager.Update()

        # Spin in place for real time to catch up
        realtime_timer.Spin(step_size)

    return 0


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')
veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')

# Initial vehicle location
initLoc = chrono.ChVectorD(-50, 0, 0.7)

# Vehicle target speed (cruise-control)
target_speed = 12

# Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
chassis_vis_type = veh.VisualizationType_NONE
suspension_vis_type =  veh.VisualizationType_PRIMITIVES
steering_vis_type = veh.VisualizationType_PRIMITIVES
wheel_vis_type = veh.VisualizationType_MESH
tire_vis_type = veh.VisualizationType_MESH 

# Type of powertrain model (SHAFTS, SIMPLE)
powertrain_model = veh.PowertrainModelType_SHAFTS

# Drive type (FWD, RWD, or AWD)
drive_type = veh.DrivelineTypeWV_AWD

# Steering type (PITMAN_ARM or PITMAN_ARM_SHAFTS)
steering_type = veh.SteeringTypeWV_PITMAN_ARM

# Type of tire model (RIGID, RIGID_MESH, FIALA, PAC89)
tire_model = veh.TireModelType_TMEASY

# Contact method
contact_method = chrono.ChContactMethod_SMC

# Flag to activate irrlicht
USE_IRRLICHT = False

# Simulation step sizes
step_size = 2e-3;
tire_step_size = 1e-3;

# Simulation end time
t_end = 100;


main()
