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


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.sensor as sens

print ("Example: create a system and visualize it in realtime 3D");

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../../../data/")

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

sys      = chrono.ChSystemNSC()

# Create a fixed rigid body

mbody1 = chrono.ChBody()
mbody1.SetBodyFixed(True)
mbody1.SetPos( chrono.ChVectorD(0,0,-0.2))
sys.Add(mbody1)

mboxasset = chrono.ChBoxShape()
mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
mbody1.AddVisualShape(mboxasset)



# Create a swinging rigid body

mbody2 = chrono.ChBody()
mbody2.SetBodyFixed(False)
sys.Add(mbody2)

mboxasset = chrono.ChBoxShape()
mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
mboxasset.SetTexture('../../../data/textures/concrete.jpg')
mbody2.AddVisualShape(mboxasset)


# Create a revolute constraint

mlink = chrono.ChLinkRevolute()

    # the coordinate system of the constraint reference in abs. space:
mframe = chrono.ChFrameD(chrono.ChVectorD(0.1,0.5,0))

    # initialize the constraint telling which part must be connected, and where:
mlink.Initialize(mbody1,mbody2, mframe)

sys.Add(mlink)

#  Create an Irrlicht application to visualize the system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('PyChrono example')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.6,0.6,0.8))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

manager = sens.ChSensorManager(sys)

imu = sens.ChIMUSensor(mbody2, 2, chrono.ChFrameD(chrono.ChVectorD(0,0,0)))
imu.SetName("IMU")

SHIB = sens.SensorHostIMUBuffer()


fl = imu.FilterList()
print(type(fl))

FiA = sens.ChFilterIMUAccess()
print(type(FiA))

#imu->PushFilter(chrono_types::make_shared<ChFilterIMUAccess>())
imu.FilterList().append(FiA)


manager.AddSensor(imu)



#mybuf = rec_buf.GetData()
#print(type(mybuf))

#my_filacc = imu.FilterList()[0]
#print(type(my_filacc))


while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()

    sys.DoStepDynamics(5e-3)
    manager.Update()
    
    #rec_buf = imu.GetMostRecentIMUBuffer()
    fil0 = fl[0]
    #print(type(fil0))
    fil = imu.GetFilterIMU()
    #print(type(fil))
    SHbuf = fil.GetBuffer()
    #print(type(SHbuf))
    buf = SHbuf.GetData(0)
    #print(type(buf))
    print(buf.Yaw)
    del SHbuf
    #del buf
    
    #print(type(rec_buf))
    #mybuf = rec_buf.GetData()
    #myacc = rec_buf.GetAcc0()


#print(type(mybuf))

#mybuf = rec_buf.GetData()



