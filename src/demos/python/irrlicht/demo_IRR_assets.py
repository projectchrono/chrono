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

# Create a Chrono physical sys
sys = chrono.ChSystemNSC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Example 1:

# Create a ChBody, and attach some 'assets' that define 3D shapes for visualization purposes.
# Note: these assets are independent from collision shapes!

# Create a rigid body as usual, and add it
# to the physical sys:
floor = chrono.ChBody()
floor.SetFixed(True)


# Contact material
floor_mat = chrono.ChContactMaterialNSC()


# Define a collision shape
floor_ct_shape = chrono.ChCollisionShapeBox(floor_mat, 20, 1, 20)
floor.AddCollisionShape(floor_ct_shape, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QUNIT))
floor.EnableCollision(True)

# Add body to sys
sys.Add(floor)


# ==Asset== attach a 'box' shape.
# Note that assets are managed via shared pointer, so they can also be shared.
boxfloor = chrono.ChVisualShapeBox(20, 1, 20)
boxfloor.SetColor(chrono.ChColor(0.2, 0.3, 1.0))
floor.AddVisualShape(boxfloor, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QUNIT))


# ==Asset== attack a 'path shape populated with segments and arc fillets:
# TODO: not sure how to add them
pathfloor = chrono.ChVisualShapePath()
mseg1 = chrono.ChLineSegment(chrono.ChVector3d(1,2,0), chrono.ChVector3d(1,3,0))
mseg2 = chrono.ChLineSegment(chrono.ChVector3d(1, 3, 0), chrono.ChVector3d(2, 3, 0))
marc1 = chrono.ChLineArc(chrono.ChCoordsysd(chrono.ChVector3d(2, 3.5, 0)), 0.5, -chrono.CH_PI_2, chrono.CH_PI_2)
pathfloor.GetPathGeometry().AddSubLine(mseg1)
pathfloor.GetPathGeometry().AddSubLine(mseg2)
pathfloor.GetPathGeometry().AddSubLine(marc1)
floor.AddVisualShape(pathfloor)

# ==Asset== attach a 'nurbs line' shape":
# (first you create the ChLineNurbs geometry,
# then you put it inside a ChVisualShapeLine asset)

nurbs = chrono.ChLineNurbs()
v1 = chrono.ChVector3d(1, 2, -1)
v2 = chrono.ChVector3d(1, 3, -1)
v3 = chrono.ChVector3d(1, 3, -2)
v4 = chrono.ChVector3d(1, 4, -2)
controlpoints = chrono.vector_ChVector3d([v1, v2, v3, v4])
nurbs.Setup(3, controlpoints)

nurbsasset = chrono.ChVisualShapeLine()
nurbsasset.SetLineGeometry(nurbs)
floor.AddVisualShape(nurbsasset)


# ==Asset== attach a 'nurbs surface' shape:
# (first you create the ChSurfaceNurbs geometry,
# then you put it inside a ChVisualShapeSurface asset)
#
# NOTE: not working at this time
#       requires proper wrapping of matrix_ChVector3d...

#mlist = [[chrono.ChVector3d(1, 2, 3), chrono.ChVector3d(1, 2, 1)],
#         [chrono.ChVector3d(1, 3, 3), chrono.ChVector3d(1, 3, 1)],
#         [chrono.ChVector3d(2, 3, 3), chrono.ChVector3d(3, 3, 1)],
#         [chrono.ChVector3d(2, 4, 3), chrono.ChVector3d(2, 4, 1)]]
#surfpoints = chrono.matrix_ChVector3d()
#surfpoints.SetMatr(mlist)
#
#msurf = chrono.ChSurfaceNurbs()
#msurf.Setup(3, 1, surfpoints)
#
#msurfasset = chrono.ChVisualShapeSurface()
#msurfasset.Pos = chrono.ChVector3d(3, -1, 3)
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
body.SetFixed(True)
sys.Add(body)

# Create a shared visual material
orange_mat = chrono.ChVisualMaterial()
orange_mat.SetDiffuseColor(chrono.ChColor(0.9, 0.4, 0.2))

# ==Asset== Attach a 'sphere' shape
sphere = chrono.ChVisualShapeSphere(0.5)
sphere.AddMaterial(orange_mat)
body.AddVisualShape(sphere, chrono.ChFramed(chrono.ChVector3d(-1,0,0), chrono.QUNIT))

# ==Asset== Attach also a 'box' shape
box = chrono.ChVisualShapeBox(0.6, 1.0, 0.2)
box.AddMaterial(orange_mat)
body.AddVisualShape(box, chrono.ChFramed(chrono.ChVector3d(1,1,0), chrono.QUNIT))

# ==Asset== Attach also a 'cylinder' shape
cyl = chrono.ChVisualShapeCylinder(0.3, 0.7)
body.AddVisualShape(cyl, 
                    chrono.ChFramed(chrono.ChVector3d(2, 0.15, 0),
                                    chrono.QuatFromAngleX(chrono.CH_PI_2)))

# ==Asset== Attach three instances of the same 'triangle mesh' shape
# TODO: not sure how to add vertices
mesh = chrono.ChVisualShapeTriangleMesh()
mesh.GetMesh().AddTriangle(chrono.ChVector3d(0, 0, 0), chrono.ChVector3d(0, 1, 0), chrono.ChVector3d(1, 0, 0))
mesh.AddMaterial(orange_mat)

body.AddVisualShape(mesh, chrono.ChFramed(chrono.ChVector3d(2,0,2), chrono.QUNIT))
body.AddVisualShape(mesh, chrono.ChFramed(chrono.ChVector3d(3,0,2), chrono.QUNIT))
body.AddVisualShape(mesh, chrono.ChFramed(chrono.ChVector3d(2,1,2), chrono.QUNIT))


# ==Asset== Attach a 'Wavefront mesh' asset, referencing a .obj file and offset it.
objmesh = chrono.ChVisualShapeModelFile()
objmesh.SetFilename(chrono.GetChronoDataFile('models/forklift/body.obj'))
objmesh.SetTexture(chrono.GetChronoDataFile('textures/bluewhite.png'))
body.AddVisualShape(objmesh, chrono.ChFramed(chrono.ChVector3d(0,0,2), chrono.QUNIT))

# ==Asset== , chrono.ChFramed(chrono.ChVector3d(2,1,2), chrono.QUNIT))
for j in range(20):
    smallbox = chrono.ChVisualShapeBox(0.2, 0.2, 0.02)
    smallbox.SetColor(chrono.ChColor(j * 0.05, 1 - j * 0.05, 0.0))
    rot = chrono.ChMatrix33d(chrono.QuatFromAngleY(j * 21 * chrono.CH_DEG_TO_RAD))
    pos = rot * chrono.ChVector3d(0.4, 0, 0) + chrono.ChVector3d(0, j * 0.02, 0)
    body.AddVisualShape(smallbox, chrono.ChFramed(pos, rot))

#
# EXAMPLE 3:
#

# Create a ChParticleClones cluster, and attach 'assets'
# that define a single "sample" 3D shape. This will be shwon
# N times in Irrlicht.

# Create the ChParticleCloud, populate it with random particles,
# and add it to physical sys:
particles = chrono.ChParticleCloud()

# Note: coll. shape, if needed, must be specified before creating particles.
# This will be shared among all particles in the ChParticleCloud.
particle_mat = chrono.ChContactMaterialNSC()

particles_ct_shape = chrono.ChCollisionShapeSphere(particle_mat, 0.05)
particles.AddCollisionShape(particles_ct_shape)
particles.EnableCollision(True)

# Create the random particles
for i in range(100):
    particles.AddParticle(chrono.ChCoordsysd(chrono.ChVector3d(chrono.ChRandom.Get() - 2, 1.5, chrono.ChRandom.Get() + 2)))

# Mass and inertia properties.
# This will be shared among all particles in the ChParticleCloud.
particles.SetMass(0.1)
particles.SetInertiaXX(chrono.ChVector3d(0.001, 0.001, 0.001))

# Do not forget to add the particles cluster to the sys
sys.Add(particles)

# ==Asset== Attach a 'sphere' shape asset. it will be used as a sample
# shape to display all particles when rendering in 3D!
sphereparticle = chrono.ChVisualShapeSphere(0.05)
particles.AddVisualShape(sphereparticle)
 
displ = chrono.ChVector3d(1.0, 0.0, 0.0)
v1 = chrono.ChVector3d(0.8, 0.0, 0.0) + displ
v2 = chrono.ChVector3d(0.8, 0.3, 0.0) + displ
v3 = chrono.ChVector3d(0.8, 0.3, 0.3) + displ
v4 = chrono.ChVector3d(0.0, 0.3, 0.3) + displ
v5 = chrono.ChVector3d(0.0, 0.0, 0.3) + displ
v6 = chrono.ChVector3d(0.8, 0.0, 0.3) + displ
mpoints = chrono.vector_ChVector3d([v1 , v2, v3, v4, v5, v6])

hull = chrono.ChBodyEasyConvexHullAuxRef(mpoints, 1000, True, True, chrono.ChContactMaterialNSC())
 
hull.Move(chrono.ChVector3d(2, 0.3, 0))
sys.Add(hull)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Chrono::Irrlicht visualization')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-2, 3, -4))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.GetGUIEnvironment().addStaticText('Hello World!', chronoirr.recti(50, 60, 150, 80))
    vis.EndScene()
    sys.DoStepDynamics(0.01)

