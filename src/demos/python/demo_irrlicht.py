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
import ChronoEngine_PYTHON_core as chrono
import ChronoEngine_PYTHON_postprocess as postprocess
import ChronoEngine_PYTHON_irrlicht as chronoirr

print ("Example: create a system and visualize it in realtime 3D");


# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystem()

# Create a rigid body, with a collision shape too

mbody1 = chrono.ChBodyShared()
mbody1.SetBodyFixed(False)
mysystem.Add(mbody1)

mboxasset = chrono.ChBoxShapeShared()
mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
mbody1.AddAsset(mboxasset)




# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

#chronoirr.ChIrrAssetConverter.SetDefaultObjectDir('../../../data/')

myapplication = chronoirr.ChIrrApp(mysystem)

myapplication.AddTypicalSky('../../../data/skybox/')
#myapplication.AddTypicalLogo('../../../data/logo_chronoengine_alpha.png')
myapplication.AddTypicalCamera()
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





