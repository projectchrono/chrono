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

print ("Example: create OpenCascade shapes and use them as rigid bodies");


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.cascade as cascade
from OCC.Core import BRepPrimAPI
from OCC.Core import BRepAlgoAPI
    

    
# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')


# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

    
sys      = chrono.ChSystemNSC()


# Set the global collision margins. This is expecially important for very large or
# very small objects. Set this before creating shapes. Not before creating sys.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001);
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001);


# A collision material, will be used by two colliding shapes
my_material = chrono.ChMaterialSurfaceNSC()
my_material.SetFriction(0.5)

# create a 3dCAD shape using the OCC OpenCascade API (a torus cut by a cylinder)
my_torus    = BRepPrimAPI.BRepPrimAPI_MakeTorus(0.1,0.02).Shape()
my_cylinder = BRepPrimAPI.BRepPrimAPI_MakeCylinder(0.09,0.1).Shape()
my_shape    = BRepAlgoAPI.BRepAlgoAPI_Cut(my_torus, my_cylinder).Shape()

# use it to make a body with proper center of mass and inertia tensor,
# given the CAD shape. Also visualize it.
my_tolerance = cascade.ChCascadeTriangulateTolerances(0.1 ,True,0.5)


my_body = cascade.ChBodyEasyCascade(my_shape,# the CAD shape
                                  1000,             # the density
                                  my_tolerance,      # must visualize triangle mesh geometry?
                                  True,              # must collide?
                                  my_material)       # collision material
sys.Add(my_body)

    
# Create a large cube as a floor.

my_floor = chrono.ChBodyEasyBox(1, 0.2, 1, # x y z size
                                1000,       # density
                                True,       # must visualize?
                                True,       # must collide?
                                my_material) # collision material
my_floor.SetPos(chrono.ChVectorD(0,-0.3,0))
my_floor.SetBodyFixed(True)
mfloor.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.5))
sys.Add(my_floor)



# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

vis = chronoirr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Use OpenCascade shapes')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.2,0.2,-0.2))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

sys.SetSolverType(chrono.ChSolver.Type_PSOR)

while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(0.005)





