#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Lijing Yang
#
# Created:     6/12/2020
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

print ("Example: demonstration of using friction models")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
# chrono.SetChronoDataPath('relative/path/to/data/directory/')

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mphysicalSystem      = chrono.ChSystemNSC()

# Create all the rigid bodies.
mradius = 0.5
density = 1000

# Create a texture asset. It can be shared between bodies.
textureasset = chrono.ChTexture(chrono.GetChronoDataFile("bluwhite.png"))

# Create some spheres that roll horizontally, with increasing rolling friction values
for bi in range(10):
    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.4)
    mat.SetRollingFriction((float(bi) / 10.) * 0.05)

    msphereBody = chrono.ChBodyEasySphere(mradius,  # radius size
                                          1000,     # density
                                          True,     # visualization?
                                          True,     # collision?
                                          mat)      # contact material

    # Set some properties
    msphereBody.SetPos(chrono.ChVectorD(-7, mradius - 0.5, -5 + bi * mradius * 2.5))
    msphereBody.AddAsset(textureasset)  # assets can be shared

    # Set initial speed: rolling in horizontal direction
    initial_angspeed = 10
    initial_linspeed = initial_angspeed * mradius
    msphereBody.SetWvel_par(chrono.ChVectorD(0, 0, -initial_angspeed))
    msphereBody.SetPos_dt(chrono.ChVectorD(initial_linspeed, 0, 0))

    # Add to the system
    mphysicalSystem.Add(msphereBody)

# Create some spheres that spin on place, for a 'drilling friction' case, with increasing spinning friction values
for bi in range(10):
    mat = chrono.ChMaterialSurfaceNSC()
    mat.SetFriction(0.4)
    mat.SetSpinningFriction((float(bi) / 10) * 0.02)

    msphereBody = chrono.ChBodyEasySphere(mradius,  # radius size
                                          1000,     # density
                                          True,     # visualization?
                                          True,     # collision?
                                          mat)      # contact material
    # Set some properties
    msphereBody.SetPos(chrono.ChVectorD(-8, 1 + mradius - 0.5, -5 + bi * mradius * 2.5))
    msphereBody.AddAsset(textureasset)  # assets can be shared

    # Set initial speed: spinning in vertical direction
    msphereBody.SetWvel_par(chrono.ChVectorD(0, 20, 0))

    # Add to the system
    mphysicalSystem.Add(msphereBody)

    # Notes:
    # - setting nonzero spinning friction and/or setting nonzero rolling friction
    #   affects the speed of the solver (each contact eats 2x of CPU time repsect to the
    #   case of simple sliding/staic contact)
    # - avoid using zero spinning friction with nonzero rolling friction.

# Create a container fixed to ground
bin = chrono.ChBody()
bin.SetPos(chrono.ChVectorD(0, -1, 0))
bin.SetBodyFixed(True)
bin.SetCollide(True)

# Set rolling and spinning friction coefficients for the container.
# By default, the composite material will use the minimum value for an interacting collision pair.
bin_mat = chrono.ChMaterialSurfaceNSC()
bin_mat.SetRollingFriction(1)
bin_mat.SetSpinningFriction(1)

# Add collision geometry and visualization shapes for the floor and the 4 walls
bin.GetCollisionModel().ClearModel()
bin.GetCollisionModel().AddBox(bin_mat, 20. / 2., 1. / 2., 20. / 2., chrono.ChVectorD(0, 0, 0))
bin.GetCollisionModel().AddBox(bin_mat, 1. / 2.,  2. / 2., 20.99/2., chrono.ChVectorD(-10, 1, 0))
bin.GetCollisionModel().AddBox(bin_mat, 1. / 2.,  2. / 2., 20.99/2., chrono.ChVectorD( 10, 1, 0))
bin.GetCollisionModel().AddBox(bin_mat, 20.99/2., 2. / 2., 1. / 2., chrono.ChVectorD(0, 1, -10))
bin.GetCollisionModel().AddBox(bin_mat, 20.99/2., 2. / 2., 1. / 2., chrono.ChVectorD(0, 1,  10))
bin.GetCollisionModel().BuildModel()

vshape_1 = chrono.ChBoxShape()
vshape_1.GetBoxGeometry().SetLengths(chrono.ChVectorD(20, 1, 20))
vshape_1.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, 0)
bin.AddAsset(vshape_1)

vshape_2 = chrono.ChBoxShape()
vshape_2.GetBoxGeometry().SetLengths(chrono.ChVectorD(1, 2, 20.99))
vshape_2.GetBoxGeometry().Pos = chrono.ChVectorD(-10, 1, 0)
bin.AddAsset(vshape_2)

vshape_3 = chrono.ChBoxShape()
vshape_3.GetBoxGeometry().SetLengths(chrono.ChVectorD(1, 2, 20.99))
vshape_3.GetBoxGeometry().Pos = chrono.ChVectorD(10, 1, 0)
bin.AddAsset(vshape_3)

vshape_4 = chrono.ChBoxShape()
vshape_4.GetBoxGeometry().SetLengths(chrono.ChVectorD(20.99, 2, 1))
vshape_4.GetBoxGeometry().Pos = chrono.ChVectorD(0, 1, -10)
bin.AddAsset(vshape_4)

vshape_5 = chrono.ChBoxShape()
vshape_5.GetBoxGeometry().SetLengths(chrono.ChVectorD(20.99, 2, 1))
vshape_5.GetBoxGeometry().Pos = chrono.ChVectorD(0, 1, 10)
bin.AddAsset(vshape_5)
bin.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("blu.png")))

mphysicalSystem.Add(bin)

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mphysicalSystem, 'PyChrono example: Friction', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(0, 14, -20))
myapplication.AddTypicalLights(chronoirr.vector3df(30., 100., 30.),
                               chronoirr.vector3df(-30, 100., 30.))

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

mphysicalSystem.SetSolverType(chrono.ChSolver.Type_APGD)
myapplication.SetTimestep(0.005)
myapplication.SetTryRealtime(True)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()





