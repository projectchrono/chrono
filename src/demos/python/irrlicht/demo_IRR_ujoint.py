#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Lijing Yang
#
# Created:     6/10/2020
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

print ("Example: demonstration of a universal joint")

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

# Set the half-length of the two shafts
hl = 2
 
# Set the bend angle between the two shafts (positive rotation
# about the global X axis)
angle = math.pi / 6.
cosa  = math.cos(angle)
sina  = math.sin(angle)
rot   = chrono.Q_from_AngX(angle) 

# Create the ground body
# ----------------------
ground = chrono.ChBody()
ground.SetIdentifier(-1)
ground.SetBodyFixed(True)
ground.SetCollide(False)
mysystem.Add(ground)

# attach visualization assets to represent the revolute and cylindrical
# joints that connect the two shafts to ground
cyl_1 = chrono.ChCylinderShape()
cyl_1.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0, -hl - 0.2)
cyl_1.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0, -hl + 0.2)
cyl_1.GetCylinderGeometry().rad = 0.3
ground.AddAsset(cyl_1)

cyl_2 = chrono.ChCylinderShape()
cyl_2.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -(hl - 0.2) * sina, (hl - 0.2) * cosa)
cyl_2.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -(hl + 0.2) * sina, (hl + 0.2) * cosa)
cyl_2.GetCylinderGeometry().rad = 0.3
ground.AddAsset(cyl_2)


# Create the first shaft body
# ---------------------------
shaft_1 = chrono.ChBody()
mysystem.AddBody(shaft_1)
shaft_1.SetIdentifier(1)
shaft_1.SetBodyFixed(False)
shaft_1.SetCollide(False)
shaft_1.SetMass(1)
shaft_1.SetInertiaXX(chrono.ChVectorD(1, 1, 0.2))
shaft_1.SetPos(chrono.ChVectorD(0, 0, -hl))
shaft_1.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))

# Add visualization assets to represent the shaft (a box) and the arm of the
# universal joint's cross associated with this shaft (a cylinder)
box_1 = chrono.ChBoxShape()
box_1.GetBoxGeometry().Size = chrono.ChVectorD(0.15, 0.15, 0.9 * hl)
shaft_1.AddAsset(box_1)

cyl_2 = chrono.ChCylinderShape()
cyl_2.GetCylinderGeometry().p1 = chrono.ChVectorD(-0.2, 0, hl)
cyl_2.GetCylinderGeometry().p2 = chrono.ChVectorD(0.2, 0, hl)
cyl_2.GetCylinderGeometry().rad = 0.05
shaft_1.AddAsset(cyl_2)

col = chrono.ChColorAsset()
col.SetColor(chrono.ChColor(0.9, 0.4, 0.1))
shaft_1.AddAsset(col)

# Create the second shaft body
# ----------------------------

# The second shaft is identical to the first one, but initialized at an angle
# equal to the specified bend angle.

shaft_2 = chrono.ChBody()
mysystem.AddBody(shaft_2)
shaft_2.SetIdentifier(1)
shaft_2.SetBodyFixed(False)
shaft_2.SetCollide(False)
shaft_2.SetMass(1)
shaft_2.SetInertiaXX(chrono.ChVectorD(1, 1, 0.2))
shaft_2.SetPos(chrono.ChVectorD(0, -hl * sina, hl * cosa))
shaft_2.SetRot(rot)

# Add visualization assets to represent the shaft (a box) and the arm of the
# universal joint's cross associated with this shaft (a cylinder)
box_1 = chrono.ChBoxShape()
box_1.GetBoxGeometry().Size = chrono.ChVectorD(0.15, 0.15, 0.9 * hl)
shaft_2.AddAsset(box_1)

cyl_2 = chrono.ChCylinderShape()
cyl_2.GetCylinderGeometry().p1 = chrono.ChVectorD(0, -0.2, -hl)
cyl_2.GetCylinderGeometry().p2 = chrono.ChVectorD(0, 0.2, -hl)
cyl_2.GetCylinderGeometry().rad = 0.05
shaft_2.AddAsset(cyl_2)

col = chrono.ChColorAsset()
col.SetColor(chrono.ChColor(0.2, 0.4, 0.8))
shaft_2.AddAsset(col)

# Connect the first shaft to ground
# ---------------------------------

# Use a rotational motor to impose both the revolute joint constraints, as well
# as constant angular velocity. Here, we drive the motor angle with a ramp function.
# Alternatively, we could use a ChLinkMotorAngularSpeed with constant speed.
# The joint is located at the origin of the first shaft.
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(ground,
                 shaft_1,
                 chrono.ChFrameD(chrono.ChVectorD(0, 0, -hl), chrono.ChQuaternionD(1, 0, 0, 0)))
motor.SetAngleFunction(chrono.ChFunction_Ramp(0, 1))
mysystem.AddLink(motor)

# Connect the second shaft to ground through a cylindrical joint
# --------------------------------------------------------------

# Use a cylindrical joint so that we do not have redundant constraints
# (note that, technically Chrono could deal with a revolute joint here).
# the joint is located at the origin of the second shaft.

cyljoint = chrono.ChLinkLockCylindrical()
mysystem.AddLink(cyljoint)
cyljoint.Initialize(ground, 
                    shaft_2,
                    chrono.ChCoordsysD(chrono.ChVectorD(0, -hl * sina, hl * cosa), rot))

# Connect the two shafts through a universal joint
# ------------------------------------------------

# The joint is located at the global origin.  Its kinematic constraints will
# enforce orthogonality of the associated cross.

ujoint = chrono.ChLinkUniversal()
mysystem.AddLink(ujoint)
ujoint.Initialize(shaft_1,
                  shaft_2,
                  chrono.ChFrameD(chrono.ChVectorD(0, 0, 0), rot))

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example: universal joint', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(3, 1, -1.5))
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
myapplication.SetTimestep(0.001)
myapplication.SetTryRealtime(True)
frame = 0

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
    frame += 1

    if frame % 20 == 0:
        omega_1 = shaft_1.GetWvel_loc().z
        omega_2 = shaft_2.GetWvel_loc().z
        print('{:.4}'.format(str(mysystem.GetChTime())), '{:.6}'.format(str(omega_1)), '{:.6}'.format(str(omega_2)))






