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


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math as m


print (" Demo of using the assets sys to create shapes for Irrlicht visualization")


# The path to the Chrono directory containing various assets(meshes, textures, data files)
# is automatically set, relative to the default lcoation of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('relative/path/to/data/directory')

# Create a Chrono::Engine physical sys
sys = chrono.ChSystemNSC()

# Example 1:

# Create a ChBody, and attach some 'assets' that define 3D shapes for visualization purposes.
# Note: these assets are independent from collision shapes!

# Create a rigid body as usual, and add it
# to the physical sys:
floor = chrono.ChBody()
floor.SetBodyFixed(True)


# Contact material
floor_mat = chrono.ChMaterialSurfaceNSC()


# Define a collision shape
floor.GetCollisionModel().ClearModel()
floor.GetCollisionModel().AddBox(floor_mat, 10, 0.5, 10, chrono.ChVectorD(0, -1, 0))
floor.GetCollisionModel().BuildModel()
floor.SetCollide(True)

# Add body to sys
sys.Add(floor)


# ==Asset== attach a 'box' shape.
# Note that assets are managed via shared pointer, so they can also be shared.
boxfloor = chrono.ChBoxShape()
boxfloor.GetBoxGeometry().Size = chrono.ChVectorD(10, 0.5, 10)
boxfloor.SetColor(chrono.ChColor(0.2, 0.3, 1.0))
floor.AddVisualShape(boxfloor, chrono.ChFrameD(chrono.ChVectorD(0, -1, 0), chrono.QUNIT))


# ==Asset== attack a 'path shape populated with segments and arc fillets:
# TODO: not sure how to add them
pathfloor = chrono.ChPathShape()
mseg1 = chrono.ChLineSegment(chrono.ChVectorD(1,2,0), chrono.ChVectorD(1,3,0))
mseg2 = chrono.ChLineSegment(chrono.ChVectorD(1, 3, 0), chrono.ChVectorD(2, 3, 0))
marc1 = chrono.ChLineArc(chrono.ChCoordsysD(chrono.ChVectorD(2, 3.5, 0)), 0.5, -chrono.CH_C_PI_2, chrono.CH_C_PI_2)
pathfloor.GetPathGeometry().AddSubLine(mseg1)
pathfloor.GetPathGeometry().AddSubLine(mseg2)
pathfloor.GetPathGeometry().AddSubLine(marc1)
floor.AddVisualShape(pathfloor)

# ==Asset== attach a 'nurbs line' shape":
# (first you create the ChLineNurbs geometry,
# then you put it inside a ChLineShape asset)

nurbs = chrono.ChLineNurbs()
v1 = chrono.ChVectorD(1, 2, -1)
v2 = chrono.ChVectorD(1, 3, -1)
v3 = chrono.ChVectorD(1, 3, -2)
v4 = chrono.ChVectorD(1, 4, -2)
controlpoints = chrono.vector_ChVectorD([v1, v2, v3, v4])
nurbs.SetupData(3, controlpoints)

nurbsasset = chrono.ChLineShape()
nurbsasset.SetLineGeometry(nurbs)
floor.AddVisualShape(nurbsasset)


# ==Asset== attach a 'nurbs surface' shape:
# (first you create the ChSurfaceNurbs geometry,
# then you put it inside a ChSurfaceShape asset)
#
# NOTE: not working at this time
#       requires proper wrapping of matrix_ChVectorD...

#mlist = [[chrono.ChVectorD(1, 2, 3), chrono.ChVectorD(1, 2, 1)],
#         [chrono.ChVectorD(1, 3, 3), chrono.ChVectorD(1, 3, 1)],
#         [chrono.ChVectorD(2, 3, 3), chrono.ChVectorD(3, 3, 1)],
#         [chrono.ChVectorD(2, 4, 3), chrono.ChVectorD(2, 4, 1)]]
#surfpoints = chrono.matrix_ChVectorD()
#surfpoints.SetMatr(mlist)
#
#msurf = chrono.ChSurfaceNurbs()
#msurf.SetupData(3, 1, surfpoints)
#
#msurfasset = chrono.ChSurfaceShape()
#msurfasset.Pos = chrono.ChVectorD(3, -1, 3)
#msurfasset.SetSurfaceGeometry(msurf)
#msurfasset.SetWireframe(True)
#floor.AddVisualShape(msurfasset)


# 
# Example 2:
#

# Textures, colors, asset levels with transformations.
# This section shows how to add more advanced typers of assets
# and how to group assets in ChAssetLevel containers.

# Create the rigid body as usual (this won't move, it is only for visualization tests)
body = chrono.ChBody()
body.SetBodyFixed(True)
sys.Add(body)

# Create a shared visual material
orange_mat = chrono.ChVisualMaterial()
orange_mat.SetDiffuseColor(chrono.ChColor(0.9, 0.4, 0.2))

# ==Asset== Attach a 'sphere' shape
sphere = chrono.ChSphereShape()
sphere.GetSphereGeometry().rad = 0.5
sphere.AddMaterial(orange_mat)
body.AddVisualShape(sphere, chrono.ChFrameD(chrono.ChVectorD(-1,0,0), chrono.QUNIT))

# ==Asset== Attach also a 'box' shape
box = chrono.ChBoxShape()
box.GetBoxGeometry().Size = chrono.ChVectorD(0.3, 0.5, 0.1)
box.AddMaterial(orange_mat)
body.AddVisualShape(box, chrono.ChFrameD(chrono.ChVectorD(1,1,0), chrono.QUNIT))

# ==Asset== Attach also a 'cylinder' shape
cyl = chrono.ChCylinderShape()
cyl.GetCylinderGeometry().p1 = chrono.ChVectorD(2, -0.2, 0)
cyl.GetCylinderGeometry().p2 = chrono.ChVectorD(2.2, 0.5, 0)
cyl.GetCylinderGeometry().rad = 0.3
body.AddVisualShape(cyl)

# ==Asset== Attach three instances of the same 'triangle mesh' shape
# TODO: not sure how to add vertices
mesh = chrono.ChTriangleMeshShape()
mesh.GetMesh().addTriangle(chrono.ChVectorD(0, 0, 0), chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(1, 0, 0))
mesh.AddMaterial(orange_mat)

body.AddVisualShape(mesh, chrono.ChFrameD(chrono.ChVectorD(2,0,2), chrono.QUNIT))
body.AddVisualShape(mesh, chrono.ChFrameD(chrono.ChVectorD(3,0,2), chrono.QUNIT))
body.AddVisualShape(mesh, chrono.ChFrameD(chrono.ChVectorD(2,1,2), chrono.QUNIT))


# ==Asset== Attach a 'Wavefront mesh' asset, referencing a .obj file and offset it.
objmesh = chrono.ChObjShapeFile()
objmesh.SetFilename(chrono.GetChronoDataFile('models/forklift/body.obj'))
objmesh.SetTexture(chrono.GetChronoDataFile('textures/bluewhite.png'))
body.AddVisualShape(objmesh, chrono.ChFrameD(chrono.ChVectorD(0,0,2), chrono.QUNIT))

# ==Asset== , chrono.ChFrameD(chrono.ChVectorD(2,1,2), chrono.QUNIT))
for j in range(20):
    smallbox = chrono.ChBoxShape()
    smallbox.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.01)
    smallbox.SetColor(chrono.ChColor(j * 0.05, 1 - j * 0.05, 0.0))
    rot = chrono.ChMatrix33D(chrono.Q_from_AngY(j * 21 * chrono.CH_C_DEG_TO_RAD))
    pos = rot * chrono.ChVectorD(0.4, 0, 0) + chrono.ChVectorD(0, j * 0.02, 0)
    body.AddVisualShape(smallbox, chrono.ChFrameD(pos, rot))

#
# EXAMPLE 3:
#

# Create a ChParticleClones cluster, and attach 'assets'
# that define a single "sample" 3D shape. This will be shwon
# N times in Irrlicht.

# Create the ChParticlesClones, populate it with random particles,
# and add it to physical sys:
particles = chrono.ChParticlesClones()

# Note: coll. shape, if needed, must be specified before creating particles.
# This will be shared among all particles in the ChParticlesClones.
particle_mat = chrono.ChMaterialSurfaceNSC()

particles.GetCollisionModel().ClearModel()
particles.GetCollisionModel().AddSphere(particle_mat, 0.05)
particles.GetCollisionModel().BuildModel()
particles.SetCollide(True)

# Create the random particles
for i in range(100):
    particles.AddParticle(chrono.ChCoordsysD(chrono.ChVectorD(chrono.ChRandom() - 2, 1.5, chrono.ChRandom() + 2)))

# Mass and inertia properties.
# This will be shared among all particles in the ChParticlesClones.
particles.SetMass(0.1)
particles.SetInertiaXX(chrono.ChVectorD(0.001, 0.001, 0.001))

# Do not forget to add the particles cluster to the sys
sys.Add(particles)

# ==Asset== Attach a 'sphere' shape asset. it will be used as a sample
# shape to display all particles when rendering in 3D!
sphereparticle = chrono.ChSphereShape()
sphereparticle.GetSphereGeometry().rad = 0.05
particles.AddVisualShape(sphereparticle)
 
displ = chrono.ChVectorD(1.0, 0.0, 0.0)
v1 = chrono.ChVectorD(0.8, 0.0, 0.0) + displ
v2 = chrono.ChVectorD(0.8, 0.3, 0.0) + displ
v3 = chrono.ChVectorD(0.8, 0.3, 0.3) + displ
v4 = chrono.ChVectorD(0.0, 0.3, 0.3) + displ
v5 = chrono.ChVectorD(0.0, 0.0, 0.3) + displ
v6 = chrono.ChVectorD(0.8, 0.0, 0.3) + displ
mpoints = chrono.vector_ChVectorD([v1 , v2, v3, v4, v5, v6])

hull = chrono.ChBodyEasyConvexHullAuxRef(mpoints, 1000, True, True, chrono.ChMaterialSurfaceNSC())
 
hull.Move(chrono.ChVectorD(2, 0.3, 0))
sys.Add(hull)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Chrono::Irrlicht visualization')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(-2, 3, -4))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.DrawAll()
    vis.GetGUIEnvironment().addStaticText('Hello World!', chronoirr.recti(50, 60, 150, 80))
    vis.EndScene()
    sys.DoStepDynamics(0.01)

