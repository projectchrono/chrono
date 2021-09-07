import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import random

class simulation:

    def __init__(self) -> None:
        self.system = chrono.ChSystemNSC()
        self.Set_G_acc(chrono.ChVectorD(0,0,0))

        green = self.init_vis_mat(chrono.ChVectorF(0,1,0))
        red = self.init_vis_mat(chrono.ChVectorF(1,0,0))

        ground = chrono.ChBodyEasyBox(1000,20,1,1000,True,False)
        ground.SetPos(chrono.ChVectord(0,0,-1))
        ground.SetBodyFixed(True)
        self.system.Add(ground)

        egocar = chrono.ChBodyEasyBox(5,2,2,True,False)
        egocar.SetPos(chrono.ChVector(0,0,0))
        self.system.Add(egocar)

        # incoming cars
        # cars in same lane 
        # cars in right lane moving in same direction


    # color should be a chrono.ChVectorF(float,float,float)
    def init_vis_mat(self, color):
        vis_mat = chrono.ChVisualMaterial()
        vis_mat.SetDiffuseColor(color)
        vis_mat.SetSpecularColor(chrono.ChVectorF(1,1,1))
        return vis_mat

    def adding_sensors(self, body, offset_pose):
        self.manager = sens.ChSensorManager(self.system)
        intensity = 1.0
        self.manager.scene.AddPointLight(chrono.ChVectorF(
            2, 2.5, 100), chrono.ChVectorF(intensity, intensity, intensity), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(
            9, 2.5, 100), chrono.ChVectorF(intensity, intensity, intensity), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(
            16, 2.5, 100), chrono.ChVectorF(intensity, intensity, intensity), 500.0)
        self.manager.scene.AddPointLight(chrono.ChVectorF(
            23, 2.5, 100), chrono.ChVectorF(intensity, intensity, intensity), 500.0)
       
        update_rate = 30
        lag = 0
        exposure_time = 0
        image_width = 1280
        image_height = 720
        fov = 1.408 # camera's horizontal field of view
        self.cam = sens.ChCameraSensor(body, update_rate,offset_pose,image_width,image_height,fov)
        self.cam.SetName("Camera Sensor")
        self.cam.SetLag(lag)
        self.cam.SetCollectionWindow(exposure_time)
        self.cam.PushFilter(sens.ChFilterRGBA8Access())
        self.manager.AddSensor(self.cam)
    
    def sim_advance(self):
        step_size = 1e-3
        self.manager.Update()
        self.system.DoStepDynamics(step_size)






def main():

