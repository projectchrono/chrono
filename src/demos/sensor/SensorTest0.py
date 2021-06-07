#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Alessandro Tasora
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


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

mysystem      = chrono.ChSystemNSC()

# Create a fixed rigid body

mbody1 = chrono.ChBody()
mbody1.SetBodyFixed(True)
mbody1.SetPos( chrono.ChVectorD(0,0,-0.2))
mysystem.Add(mbody1)

mboxasset = chrono.ChBoxShape()
mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
mbody1.AddAsset(mboxasset)



# Create a swinging rigid body

mbody2 = chrono.ChBody()
mbody2.SetBodyFixed(False)
mysystem.Add(mbody2)

mboxasset = chrono.ChBoxShape()
mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
mbody2.AddAsset(mboxasset)

mboxtexture = chrono.ChTexture()
mboxtexture.SetTextureFilename('../../../data/textures/concrete.jpg')
mbody2.GetAssets().push_back(mboxtexture)


# Create a revolute constraint

mlink = chrono.ChLinkRevolute()

    # the coordinate system of the constraint reference in abs. space:
mframe = chrono.ChFrameD(chrono.ChVectorD(0.1,0.5,0))

    # initialize the constraint telling which part must be connected, and where:
mlink.Initialize(mbody1,mbody2, mframe)

mysystem.Add(mlink)

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo()
myapplication.AddTypicalCamera(chronoirr.vector3df(0.6,0.6,0.8))
myapplication.AddTypicalLights()

            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			# If you need a finer control on which item really needs a visualization proxy in
			# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

myapplication.AssetBindAll();

			# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll();


# ---------------------------------------------------------------------
#
#  Run the simulation
#


myapplication.SetTimestep(0.005)

manager = sens.ChSensorManager(mysystem)

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


while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    manager.Update()
    myapplication.EndScene()
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



