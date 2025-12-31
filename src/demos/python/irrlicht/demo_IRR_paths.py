# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

print ("Example: create a sys and visualize it in realtime 3D");

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys      = chrono.ChSystemNSC()

mfloor = chrono.ChBodyEasyBox(3, 0.2, 3, 1000)
mfloor.SetFixed(True)
sys.Add(mfloor)

#
# EXAMPLE1
#

# Create a ChLinePath geometry, and insert sub-paths:
mpath = chrono.ChLinePath()
mseg1 = chrono.ChLineSegment(chrono.ChVector3d(1, 2, 0), chrono.ChVector3d(2, 2, 0))
mpath.AddSubLine(mseg1)
marc1 = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(2, 2.5, 0)), 0.5, -chrono.CH_PI_2, chrono.CH_PI_2, True)
mpath.AddSubLine(marc1)
mseg2 = chrono.ChLineSegment(chrono.ChVector3d(2, 3, 0), chrono.ChVector3d(1, 3, 0))
mpath.AddSubLine(mseg2)
marc2 = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(1, 2.5, 0)), 0.5, chrono.CH_PI_2, -chrono.CH_PI_2, True);
mpath.AddSubLine(marc2)
mpath.SetClosed(True)

# Create a ChVisualShapeLine, a visualization asset for lines.
# The ChLinePath is a special type of ChLine and it can be visualized.
mpathasset = chrono.ChVisualShapeLine()
mpathasset.SetLineGeometry(mpath)
mfloor.AddVisualShape(mpathasset)

# Create a body that will follow the trajectory

mpendulum = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000)
mpendulum.SetPos(chrono.ChVector3d(1, 1.5, 0))
sys.Add(mpendulum)

# The trajectory constraint:

mtrajectory = chrono.ChLinkLockTrajectory()

# Define which parts are connected (the trajectory is considered in the 2nd body).
mtrajectory.Initialize(mpendulum,                    # body1 that follows the trajectory
                      mfloor,                        # body2 that 'owns' the trajectory
                      chrono.ChVector3d(0, 0.5, 0),  # point on body1 that will follow the trajectory
                      mpath                          # the trajectory (reuse the one already added to body2 as asset)
                      )

# Optionally, set a function that gets the curvilinear
# abscyssa s of the line, as a function of time s(t). 
# By default it was simply  s=t.
mspacefx = chrono.ChFunctionRamp(0, 0.5)
mtrajectory.SetTimeLaw(mspacefx)

sys.Add(mtrajectory)

#
# EXAMPLE 2:
#

# Create a ChBody that contains the trajectory

mwheel = chrono.ChBody()
mwheel.SetPos(chrono.ChVector3d(-3, 2, 0))
sys.Add(mwheel)

# Create a motor that spins the wheel
my_motor = chrono.ChLinkMotorRotationSpeed()
my_motor.Initialize(mwheel, mfloor, chrono.ChFramed(chrono.ChVector3d(-3, 2, 0)))
my_angularspeed = chrono.ChFunctionConst(chrono.CH_PI / 4.0)
my_motor.SetMotorFunction(my_angularspeed)
sys.Add(my_motor)

# Create a ChLinePath geometry, and insert sub-paths:
mglyph = chrono.ChLinePath()
ms1 = chrono.ChLineSegment(chrono.ChVector3d(-0.5, -0.5, 0), chrono.ChVector3d(0.5, -0.5, 0))
mglyph.AddSubLine(ms1)
ma1 = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(0.5, 0, 0)), 0.5, -chrono.CH_PI_2, chrono.CH_PI_2, True)
mglyph.AddSubLine(ma1)
ms2 = chrono.ChLineSegment(chrono.ChVector3d(0.5, 0.5, 0), chrono.ChVector3d(-0.5, 0.5, 0))
mglyph.AddSubLine(ms2)
ma2 = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(-0.5, 0, 0)), 0.5, chrono.CH_PI_2, -chrono.CH_PI_2, True);
mglyph.AddSubLine(ma2)
mglyph.SetPathDuration(1)
mglyph.SetClosed(True)

# Create a ChVisualShapeLine, a visualization asset for lines.
# The ChLinePath is a special type of ChLine and it can be visualized.
mglyphasset = chrono.ChVisualShapeLine()
mglyphasset.SetLineGeometry(mglyph)
mwheel.AddVisualShape(mglyphasset)

# Create a body that will slide on the glyph

mpendulum2 = chrono.ChBodyEasyBox(0.1, 1, 0.1, 1000)
mpendulum2.SetPos(chrono.ChVector3d(-3, 1, 0))
sys.Add(mpendulum2)

# The glyph constraint:

mglyphconstraint = chrono.ChLinkLockPointSpline()

# Define which parts are connected (the trajectory is considered in the 2nd body).
mglyphconstraint.Initialize(mpendulum2,  # body1 that follows the trajectory
                            mwheel,      # body2 that 'owns' the trajectory
                            True,
                            chrono.ChFramed(chrono.ChVector3d(0, 0.5, 0)), # point on body1 that will follow the trajectory
                            chrono.ChFramed())

mglyphconstraint.SetTrajectory(mglyph)

sys.Add(mglyphconstraint)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Paths demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(1,4,5), chrono.ChVector3d(0,2,0))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(5e-3)





