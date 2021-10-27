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
import matplotlib.pyplot as plt
import numpy as np

print ("Example: create a slider crank and plot results");

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystemNSC()

# Some data shared in the following
crank_center = chrono.ChVectorD(-1,0.5,0)
crank_rad    = 0.4
crank_thick  = 0.1
rod_length   = 1.5


# Create four rigid bodies: the truss, the crank, the rod, the piston.

# Create the floor truss
mfloor = chrono.ChBodyEasyBox(3, 1, 3, 1000)
mfloor.SetPos(chrono.ChVectorD(0,-0.5,0))
mfloor.SetBodyFixed(True)
mysystem.Add(mfloor)

# Create the flywheel crank
mcrank = chrono.ChBodyEasyCylinder(crank_rad, crank_thick, 1000)
mcrank.SetPos(crank_center + chrono.ChVectorD(0, 0, -0.1))
# Since ChBodyEasyCylinder creates a vertical (y up) cylinder, here rotate it:
mcrank.SetRot(chrono.Q_ROTATE_Y_TO_Z)
mysystem.Add(mcrank)

# Create a stylized rod
mrod = chrono.ChBodyEasyBox(rod_length, 0.1, 0.1, 1000)
mrod.SetPos(crank_center + chrono.ChVectorD(crank_rad+rod_length/2 , 0, 0))
mysystem.Add(mrod)

# Create a stylized piston
mpiston = chrono.ChBodyEasyCylinder(0.2, 0.3, 1000)
mpiston.SetPos(crank_center + chrono.ChVectorD(crank_rad+rod_length, 0, 0))
mpiston.SetRot(chrono.Q_ROTATE_Y_TO_X)
mysystem.Add(mpiston)


# Now create constraints and motors between the bodies.

# Create crank-truss joint: a motor that spins the crank flywheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mcrank,   # the first connected body
                    mfloor,   # the second connected body
                    chrono.ChFrameD(crank_center)) # where to create the motor in abs.space
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI) # ang.speed: 180°/s
my_motor.SetMotorFunction(my_angularspeed)
mysystem.Add(my_motor)

# Create crank-rod joint
mjointA = chrono.ChLinkLockRevolute()
mjointA.Initialize(mrod,
                   mcrank, 
                   chrono.ChCoordsysD( crank_center + chrono.ChVectorD(crank_rad,0,0) ))
mysystem.Add(mjointA)

# Create rod-piston joint
mjointB = chrono.ChLinkLockRevolute()
mjointB.Initialize(mpiston,
                   mrod, 
                   chrono.ChCoordsysD( crank_center + chrono.ChVectorD(crank_rad+rod_length,0,0) ))
mysystem.Add(mjointB)

# Create piston-truss joint
mjointC = chrono.ChLinkLockPrismatic()
mjointC.Initialize(mpiston,
                   mfloor, 
                   chrono.ChCoordsysD( 
                               crank_center + chrono.ChVectorD(crank_rad+rod_length,0,0), 
                               chrono.Q_ROTATE_Z_TO_X)
                  )
mysystem.Add(mjointC)



# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(1,1,3), chronoirr.vector3df(0,1,0))
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

# Initialize these lists to store values to plot.
array_time   = []
array_angle  = []
array_pos    = []
array_speed  = []

myapplication.SetTimestep(0.005)
myapplication.SetTryRealtime(True)

# Run the interactive simulation loop
while(myapplication.GetDevice().run()):
    
    # for plotting, append instantaneous values:
    array_time.append(mysystem.GetChTime())
    array_angle.append(my_motor.GetMotorRot())
    array_pos.append(mpiston.GetPos().x)
    array_speed.append(mpiston.GetPos_dt().x)
    
    # here happens the visualization and step time integration
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()
    
    # stop simulation after 2 seconds
    if mysystem.GetChTime() > 20:
          myapplication.GetDevice().closeDevice()


# Use matplotlib to make two plots when simulation ended:
fig, (ax1, ax2) = plt.subplots(2, sharex = True)

ax1.plot(array_angle, array_pos)
ax1.set(ylabel='position [m]')
ax1.grid()

ax2.plot(array_angle, array_speed, 'r--')
ax2.set(ylabel='speed [m]',xlabel='angle [rad]')
ax2.grid()

# trick to plot \pi on x axis of plots instead of 1 2 3 4 etc.
plt.xticks(np.linspace(0, 2*np.pi, 5),['0','$\pi/2$','$\pi$','$3\pi/2$','$2\pi$'])





