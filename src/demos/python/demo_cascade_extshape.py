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
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.cascade as cascade
import OCC.Core.BRepPrimAPI

print ("Example: create OpenCascade shapes and use them as rigid bodies");

# Change this path to assets, if running from other woring dir. 
# It must point to the data folder.
chrono.SetChronoDataPath("C:/tasora/code/projectchrono/chrono/data/")

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#


mysystem      = chrono.ChSystemNSC()


# create a 3dCAD shape using the OCC OpenCascade API:
my_box = OCC.Core.BRepPrimAPI.BRepPrimAPI_MakeTorus(0.1,0.02).Shape() 

# use it to make a body with proper center of mass and inertia tensor,
# given the CAD shape. Also visualize it.
testb = cascade.ChBodyEasyCascade(my_box,  # the CAD shape
                                  1000,    # the density
                                  True,    # must collide using the triangle mesh geometry?
                                  True)    # must be visualized?
mysystem.Add(testb)


# Create a large cube as a floor.

mfloor = chrono.ChBodyEasyBox(1, 0.2, 1, 1000, True)
mfloor.SetPos(chrono.ChVectorD(0,-0.3,0))
mfloor.SetBodyFixed(True)
mysystem.Add(mfloor)

mcolor = chrono.ChColorAsset(0.2, 0.2, 0.5)
mfloor.AddAsset(mcolor)



# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'Use OpenCascade shapes', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky(chrono.GetChronoDataPath() + 'skybox/')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.2,0.2,-0.2))
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


myapplication.SetTimestep(0.01)


while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()





