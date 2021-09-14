import pychrono.core as chrono
import pychrono.sensor as sens

import numpy as np
import time
import random
import cv2

class simulation:

    def __init__(self) -> None:
        self.system = chrono.ChSystemNSC()
        self.system.Set_G_acc(chrono.ChVectorD(0,0,0))

        green = self.init_vis_mat(chrono.ChVectorF(0,1,0))
        red = self.init_vis_mat(chrono.ChVectorF(1,0,0))

        ground = chrono.ChBodyEasyBox(1000,20,1,1000,True,False)
        ground.SetPos(chrono.ChVectorD(0,0,-1))
        ground.SetBodyFixed(True)
        asset = ground.GetAssets()[0]
        visual_asset = chrono.CastToChVisualization(asset)
        visual_asset.material_list.append(green)
        self.system.Add(ground)

        egocar = chrono.ChBodyEasyBox(5,2,2,1000,True,False)
        egocar.SetPos(chrono.ChVectorD(0,1,0))
        car_asset = egocar.GetAssets()[0]
        car_visual_asset = chrono.CastToChVisualization(car_asset)
        car_visual_asset.material_list.append(red)
        self.system.Add(egocar)

        frontcar = chrono.ChBodyEasyBox(5,2,2,1000,True,False)
        frontcar.SetPos(chrono.ChVectorD(10,2,2))
        frontcar_asset = frontcar.GetAssets()[0]
        frontcar_visual_asset = chrono.CastToChVisualization(frontcar_asset)
        frontcar_visual_asset.material_list.append(red)
        self.system.Add(frontcar)

        offset_pose = chrono.ChFrameD(chrono.ChVectorD(0,0,2), chrono.Q_from_AngZ(0))
        self.adding_sensors(egocar, offset_pose)

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
        hfov = 1.408 # camera's horizontal field of view

        self.cam = sens.ChCameraSensor(body, update_rate,offset_pose,image_width,image_height,hfov)
        self.cam.SetName("Camera Sensor")
        self.cam.SetLag(lag)
        self.cam.SetCollectionWindow(exposure_time)
        self.cam.PushFilter(sens.ChFilterRGBA8Access())
        self.manager.AddSensor(self.cam)

        h_samples = 100
        v_samples = 100
        max_vert_angle = chrono.CH_C_PI / 12
        min_vert_angle = -chrono.CH_C_PI / 12

        self.radar = sens.ChRadarSensor(body,update_rate,offset_pose,h_samples,v_samples,hfov, max_vert_angle, min_vert_angle,100.0)
        self.radar.PushFilter(sens.ChFilterRadarProcess())
        self.radar.PushFilter(sens.ChFilterProcessedRadarAccess())
        self.manager.AddSensor(self.radar)


    def sim_advance(self):
        step_size = 1e-3
        self.manager.Update()
        self.system.DoStepDynamics(step_size)
        self.display_image()
    def display_image(self):
        rgba8_buffer = self.cam.GetMostRecentRGBA8Buffer()
        if rgba8_buffer.HasData():
            rgba8_data = rgba8_buffer.GetRGBA8Data()
            print(type(rgba8_data))
            print('RGBA8 buffer recieved from cam. Camera resolution: {0}x{1}'
                  .format(rgba8_buffer.Width, rgba8_buffer.Height))
            print('First Pixel: {0}'.format(rgba8_data[0, 0, :]))
            np.flip(rgba8_data)
            bgr = cv2.cvtColor(rgba8_data, cv2.COLOR_RGB2BGR)
        radar_buffer = self.radar.GetMostRecentProcessedRadarBuffer()
        if radar_buffer.HasData():
            radar_data = radar_buffer.GetProcessedRadarData()
            print(radar_data)
        if rgba8_buffer.HasData():
            cv2.imshow("window", bgr[::-1])
            cv2.waitKey()


def main():
    sim = simulation()
    while True:
        print("simulating")
        sim.sim_advance()

if __name__ == "__main__":
    main()

