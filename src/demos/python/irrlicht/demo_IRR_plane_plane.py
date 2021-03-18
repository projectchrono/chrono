#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Lijing Yang
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

print ("Example: demonstration of a plane-plane joint")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('relative/path/to/data/directory/')

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystemNSC()
mysystem.Set_G_acc(chrono.ChVectorD(0, 0, 0))

# Create the ground body
ground = chrono.ChBodyEasyBox(3, 2, 0.1, 10, True, False)
ground.SetBodyFixed(True)
mysystem.Add(ground)

# Create the sliding body
# Give an initial angular velocity
body = chrono.ChBodyEasyBox(0.5, 0.5, 0.5, 10, True, False)
body.SetBodyFixed(False)
mysystem.Add(body)
body.SetPos(chrono.ChVectorD(-1.25, -0.75, 0.1))
body.SetWvel_loc(chrono.ChVectorD(0.1, 0.1, 0.1))

body_col = chrono.ChColorAsset()
body_col.SetColor(chrono.ChColor(0.9, 0.4, 0.1))
body.AddAsset(body_col)

# Create the plane-plane constraint
# Constrain the sliding body to move and rotate in the x-y plane
# (i.e. the plane whose normal is the z-axis of the specified coord sys)
plane_plane = chrono.ChLinkLockPlanePlane()
plane_plane.Initialize(ground, 
                       body, 
                       chrono.ChCoordsysD(chrono.ChVectorD(-1.25, -0.75, 0), chrono.ChQuaternionD(1, 0, 0, 0)))
mysystem.AddLink(plane_plane)

# Create a linear spring (with default sprint & damping coefficients)
spring = chrono.ChLinkTSDA()
spring.SetSpringCoefficient(100)
spring.SetDampingCoefficient(5)
spring.Initialize(ground,
                  body,
                  True,
                  chrono.ChVectorD(0, 0, 2),
                  chrono.ChVectorD(0, 0, 0),
                  False,
                  1.9)
mysystem.AddLink(spring)

spring_col = chrono.ChColorAsset()
spring_col.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
spring.AddAsset(spring_col)
spring.AddAsset(chrono.ChPointPointSpring(0.05, 80, 15))

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example: ChLinkLockPlanePlane', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(3, 0 ,3))
myapplication.AddTypicalLights()

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.

myapplication.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll()


# ---------------------------------------------------------------------
#
#  Run the simulation
#


myapplication.SetTimestep(0.005)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    chronoirr.drawAllCOGs(mysystem, myapplication.GetVideoDriver(), 2)
    myapplication.DoStep()
    myapplication.EndScene()





