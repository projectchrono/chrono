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

print ("Example: create a system and visualize it in realtime 3D");

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystemNSC()

mfloor = chrono.ChBodyEasyBox(3, 0.2, 3, 1000)
mfloor.SetBodyFixed(True)
mysystem.Add(mfloor)

#
# EXAMPLE1
#

# Create a ChLinePath geometry, and insert sub-paths:
mpath = chrono.ChLinePath()
mseg1 = chrono.ChLineSegment(chrono.ChVectorD(1, 2, 0), chrono.ChVectorD(2, 2, 0))
mpath.AddSubLine(mseg1)
marc1 = chrono.ChLineArc(chrono.ChCoordsysD(chrono.ChVectorD(2, 2.5, 0)), 0.5, -chrono.CH_C_PI_2, chrono.CH_C_PI_2, True)
mpath.AddSubLine(marc1)
mseg2 = chrono.ChLineSegment(chrono.ChVectorD(2, 3, 0), chrono.ChVectorD(1, 3, 0))
mpath.AddSubLine(mseg2)
marc2 = chrono.ChLineArc(chrono.ChCoordsysD(chrono.ChVectorD(1, 2.5, 0)), 0.5, chrono.CH_C_PI_2, -chrono.CH_C_PI_2, True);
mpath.AddSubLine(marc2)
mpath.Set_closed(True)

# Create a ChLineShape, a visualization asset for lines.
# The ChLinePath is a special type of ChLine and it can be visualized.
mpathasset = chrono.ChLineShape()
mpathasset.SetLineGeometry(mpath)
mfloor.AddAsset(mpathasset)

# Create a body that will follow the trajectory

mpendulum = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000)
mpendulum.SetPos(chrono.ChVectorD(1, 1.5, 0))
mysystem.Add(mpendulum)

# The trajectory constraint:

mtrajectory = chrono.ChLinkTrajectory()

# Define which parts are connected (the trajectory is considered in the 2nd body).
mtrajectory.Initialize(mpendulum, # body1 that follows the trajectory
          mfloor,                 # body2 that 'owns' the trajectory
          chrono.ChVectorD(0, 0.5, 0),  # point on body1 that will follow the trajectory
          mpath                   # the trajectory (reuse the one already added to body2 as asset)
          )

# Optionally, set a function that gets the curvilinear
# abscyssa s of the line, as a function of time s(t). 
# By default it was simply  s=t.
mspacefx = chrono.ChFunction_Ramp(0, 0.5)
mtrajectory.Set_space_fx(mspacefx)

mysystem.Add(mtrajectory)

#
# EXAMPLE 2:
#

# Create a ChBody that contains the trajectory

mwheel = chrono.ChBody()
mwheel.SetPos(chrono.ChVectorD(-3, 2, 0))
mysystem.Add(mwheel)

# Create a motor that spins the wheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mwheel, mfloor, chrono.ChFrameD(chrono.ChVectorD(-3, 2, 0)))
my_angularspeed = chrono.ChFunction_Const(chrono.CH_C_PI / 4.0)
my_motor.SetMotorFunction(my_angularspeed)
mysystem.Add(my_motor)

# Create a ChLinePath geometry, and insert sub-paths:
mglyph = chrono.ChLinePath()
ms1 = chrono.ChLineSegment(chrono.ChVectorD(-0.5, -0.5, 0), chrono.ChVectorD(0.5, -0.5, 0))
mglyph.AddSubLine(ms1)
ma1 = chrono.ChLineArc(chrono.ChCoordsysD(chrono.ChVectorD(0.5, 0, 0)), 0.5, -chrono.CH_C_PI_2, chrono.CH_C_PI_2, True)
mglyph.AddSubLine(ma1)
ms2 = chrono.ChLineSegment(chrono.ChVectorD(0.5, 0.5, 0), chrono.ChVectorD(-0.5, 0.5, 0))
mglyph.AddSubLine(ms2)
ma2 = chrono.ChLineArc(chrono.ChCoordsysD(chrono.ChVectorD(-0.5, 0, 0)), 0.5, chrono.CH_C_PI_2, -chrono.CH_C_PI_2, True);
mglyph.AddSubLine(ma2)
mglyph.SetPathDuration(1)
mglyph.Set_closed(True)

# Create a ChLineShape, a visualization asset for lines.
# The ChLinePath is a special type of ChLine and it can be visualized.
mglyphasset = chrono.ChLineShape()
mglyphasset.SetLineGeometry(mglyph)
mwheel.AddAsset(mglyphasset)

# Create a body that will slide on the glyph

mpendulum2 = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000)
mpendulum2.SetPos(chrono.ChVectorD(-3, 1, 0))
mysystem.Add(mpendulum2)

# The glyph constraint:

mglyphconstraint = chrono.ChLinkPointSpline()

# Define which parts are connected (the trajectory is considered in the 2nd body).
mglyphconstraint.Initialize(mpendulum2,  # body1 that follows the trajectory
               mwheel,      # body2 that 'owns' the trajectory
               True,
               chrono.ChCoordsysD(chrono.ChVectorD(0, 0.5, 0)), # point on body1 that will follow the trajectory
               chrono.ChCoordsysD())

mglyphconstraint.Set_trajectory_line(mglyph)

mysystem.Add(mglyphconstraint)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(1,4,5), chronoirr.vector3df(0,2,0))
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
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()





