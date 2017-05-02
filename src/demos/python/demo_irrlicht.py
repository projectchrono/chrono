#-------------------------------------------------------------------------------
# Name:        modulo1
# Purpose:
#
# Author:      tasora
#
# Created:     14/02/2012
# Copyright:   (c) tasora 2012
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


import os
import math
import ChronoEngine_python_core as chrono
import ChronoEngine_python_postprocess as postprocess
import ChronoEngine_python_irrlicht as chronoirr

print ("Example: create a system and visualize it in realtime 3D");


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
mboxtexture.SetTextureFilename('../../../data/concrete.jpg')
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

myapplication = chronoirr.ChIrrApp(mysystem, 'Test', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky('../../../data/skybox/')
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


myapplication.SetTimestep(0.001)


while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()





