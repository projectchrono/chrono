#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Han Wang
#
# Created:     6/16/2020
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math as m


print (" Demo of using the assets system to create shapes for Irrlicht visualization")


# The path to the Chrono directory containing various assets(meshes, textures, data files)
# is automatically set, relative to the default lcoation of this demo.
# If running from a different directory, you must change the path to the data directory with:
# chrono.SetChronoDataPath('relative/path/to/data/directory')

# Create a Chrono::Engine physical system
mphysicalSystem = chrono.ChSystemNSC()

# Create the Irrlicht visualization (open the Irrlicht device, bind a simple UI, etc, etc)
application = chronoirr.ChIrrApp(mphysicalSystem, "Assets for Irrlicht visualization", chronoirr.dimension2du(1024, 768))

# Easy shorcuts to add camera, lights, logo, and sky in Irrlicht scene
application.AddTypicalSky()
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalCamera(chronoirr.vector3df(0, 4, -6))
application.AddTypicalLights()


# Example 1:

# Create a ChBody, and attach some 'assets' that define 3D shapes for visualization purposes.
# Note: these assets are independent from collision shapes!

# Create a rigid body as usual, and add it
# to the physical system:
mfloor = chrono.ChBody()
mfloor.SetBodyFixed(True)


# Contact material
floor_mat = chrono.ChMaterialSurfaceNSC()


# Define a collision shape
mfloor.GetCollisionModel().ClearModel()
mfloor.GetCollisionModel().AddBox(floor_mat, 10, 0.5, 10, chrono.ChVectorD(0, -1, 0))
mfloor.GetCollisionModel().BuildModel()
mfloor.SetCollide(True)

# Add body to system
mphysicalSystem.Add(mfloor)


# ==Asset== attach a 'box' shape.
# Note that assets are managed via shared pointer, so they
# can also be shared. Do not forget AddAsset() at the end!
mboxfloor = chrono.ChBoxShape()
mboxfloor.GetBoxGeometry().Size = chrono.ChVectorD(10, 0.5, 10)
mboxfloor.GetBoxGeometry().Pos = chrono.ChVectorD(0, -1, 0)
mfloor.AddAsset(mboxfloor)


# ==Asset== attach color asset
mfloorcolor = chrono.ChColorAsset()
mfloorcolor.SetColor(chrono.ChColor(0.3, 0.3, 0.6))
mfloor.AddAsset(mfloorcolor)


# ==Asset== attack a 'path shape populated with segments and arc fillets:
# TODO: not sure how to add them
mpathfloor = chrono.ChPathShape()
mseg1 = chrono.ChLineSegment(chrono.ChVectorD(1,2,0), chrono.ChVectorD(1,3,0))
mseg2 = chrono.ChLineSegment(chrono.ChVectorD(1, 3, 0), chrono.ChVectorD(2, 3, 0))
marc1 = chrono.ChLineArc(chrono.ChCoordsysD(chrono.ChVectorD(2, 3.5, 0)), 0.5, -chrono.CH_C_PI_2, chrono.CH_C_PI_2)
mpathfloor.GetPathGeometry().AddSubLine(mseg1)
mpathfloor.GetPathGeometry().AddSubLine(mseg2)
mpathfloor.GetPathGeometry().AddSubLine(marc1)
mfloor.AddAsset(mpathfloor)

# ==Asset== attach a 'nurbs line' shape":
# (first you create the ChLineNurbs geometry,
# then you put it inside a ChLineShape asset)

mnurbs = chrono.ChLineNurbs()
v1 = chrono.ChVectorD(1, 2, -1)
v2 = chrono.ChVectorD(1, 3, -1)
v3 = chrono.ChVectorD(1, 3, -2)
v4 = chrono.ChVectorD(1, 4, -2)
controlpoints = chrono.vector_ChVectorD([v1, v2, v3, v4])
mnurbs.SetupData(3, controlpoints)

mnurbsasset = chrono.ChLineShape()
mnurbsasset.SetLineGeometry(mnurbs)
mfloor.AddAsset(mnurbsasset)


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
#mfloor.AddAsset(msurfasset)


# 
# Example 2:
#

# Textures, colors, asset levels with transformations.
# This section shows how to add more advanced typers of assets
# and how to group assets in ChAssetLevel containers.

# Create the rigid body as usual (this won't move,
# it is only for visualization tests)
mbody = chrono.ChBody()
mbody.SetBodyFixed(True)
mphysicalSystem.Add(mbody)

# ==Asset== Attach a 'sphere' shape
msphere = chrono.ChSphereShape()
msphere.GetSphereGeometry().rad = 0.5
msphere.GetSphereGeometry().center = chrono.ChVectorD(-1,0,0)
mbody.AddAsset(msphere)

# ==Asset== Attach also a 'box' shape
mbox = chrono.ChBoxShape()
mbox.GetBoxGeometry().Pos = chrono.ChVectorD(1,1,0)
mbox.GetBoxGeometry().Size = chrono.ChVectorD(0.3, 0.5, 0.1)
mbody.AddAsset(mbox)

# ==Asset== Attach also a 'cylinder' shape
mcyl = chrono.ChCylinderShape()
mcyl.GetCylinderGeometry().p1 = chrono.ChVectorD(2, -0.2, 0)
mcyl.GetCylinderGeometry().p2 = chrono.ChVectorD(2.2, 0.5, 0)
mcyl.GetCylinderGeometry().rad = 0.3
mbody.AddAsset(mcyl)

# ==Asset== Attach also a 'triangle mesh' shape
# TODO: not sure how to add vertices
mmesh = chrono.ChTriangleMeshShape()
mmesh.GetMesh().addTriangle(chrono.ChVectorD(0, 1, 0), chrono.ChVectorD(0, 1, 0.5), chrono.ChVectorD(1, 1, 0))
mbody.AddAsset(mmesh)


# ==Asset== Attach color. To set colors for all assets
# in the same level, just asdd this:
mvisual = chrono.ChColorAsset()
mvisual.SetColor(chrono.ChColor(0.9, 0.4, 0.2))
mbody.AddAsset(mvisual)

# ==Asset== Attach a level that contains other assets
# Note: a ChAssetLevel can define a rotation/translation respect to paren level
# Note: a ChAssetLevel can contain colors or textures: if any, they affect only objects in the level
mlevelA = chrono.ChAssetLevel()

# ==Asset== Attach, in this level, a 'Wavefront mesh' asset,
# referencing a .obj file:
mobjmesh = chrono.ChObjShapeFile()
mobjmesh.SetFilename(chrono.GetChronoDataFile('forklift_body.obj'))
mlevelA.AddAsset(mobjmesh)

# ==Asset== Attach also a texture, taht will affect only the
# assets in mlevelA:
mtexture = chrono.ChTexture()
mtexture.SetTextureFilename(chrono.GetChronoDataFile('bluwhite.png'))
mlevelA.AddAsset(mtexture)

# Change the position of mlevelA, thus moving also its sub-assets:
mlevelA.GetFrame().SetPos(chrono.ChVectorD(0,0,2))
mbody.AddAsset(mlevelA)

# ==Asset== Attach sub level, then add to it an array of sub-levels,
# each rotated, and each containing a displaced box, thus making a
# spiral of cubes
mlevelB = chrono.ChAssetLevel()
for i in range(20):
    # ==Asset== the sub sub level..
    mlevelC = chrono.ChAssetLevel()

    # ==Asset== the contained box..
    msmallbox = chrono.ChBoxShape()
    msmallbox.GetBoxGeometry().Pos = chrono.ChVectorD(0.4, 0, 0)
    msmallbox.GetBoxGeometry().Size = chrono.ChVectorD(0.1, 0.1, 0.01)
    mlevelC.AddAsset(msmallbox)

    mrot = chrono.ChQuaternionD()
    mrot.Q_from_AngAxis(i * 21 * chrono.CH_C_DEG_TO_RAD, chrono.ChVectorD(0,1,0))
    mlevelC.GetFrame().SetRot(mrot)
    mlevelC.GetFrame().SetPos(chrono.ChVectorD(0,i * 0.02, 0))

    mlevelB.AddAsset(mlevelC)

mbody.AddAsset(mlevelB)

# ==Asset== Attach a video camera. This will be used by Irrlicht, 
# or POVray postprocessing, etc. Note that a camera can also be 
# put in a moving object
mcamera = chrono.ChCamera()
mcamera.SetAngle(50)
mcamera.SetPosition(chrono.ChVectorD(-3, 4, -5))
mcamera.SetAimPoint(chrono.ChVectorD(0, 1, 0))
mbody.AddAsset(mcamera)

#
# EXAMPLE 3:
#

# Create a ChParticleClones cluster, and attach 'assets'
# that define a single "sample" 3D shape. This will be shwon
# N times in Irrlicht.
# ***NOTE*** This crashes with Irrlicht 1.8, it is ok with 1.7.x and 1.8.1 +,

# Create the ChParticlesClones, populate it with random particles,
# and add it to physical system:
mparticles = chrono.ChParticlesClones()

# Note: coll. shape, if needed, must be specified before creating particles.
# This will be shared among all particles in the ChParticlesClones.
particle_mat = chrono.ChMaterialSurfaceNSC()

mparticles.GetCollisionModel().ClearModel()
mparticles.GetCollisionModel().AddSphere(particle_mat, 0.05)
mparticles.GetCollisionModel().BuildModel()
mparticles.SetCollide(True)

# Create the random particles
for i in range(100):
    mparticles.AddParticle(chrono.ChCoordsysD(chrono.ChVectorD(chrono.ChRandom() - 2, 1.5, chrono.ChRandom() + 2)))

# Mass and inertia properties.
# This will be shared among all particles in the ChParticlesClones.
mparticles.SetMass(0.1)
mparticles.SetInertiaXX(chrono.ChVectorD(0.001, 0.001, 0.001))

# Do not forget to add the particles cluster to the system
mphysicalSystem.Add(mparticles)

# ==Asset== Attach a 'sphere' shape asset. it will be used as a sample
# shape to display all particles when rendering in 3D!
mspherepart = chrono.ChSphereShape()
mspherepart.GetSphereGeometry().rad = 0.05
mparticles.AddAsset(mspherepart)
 
displ = chrono.ChVectorD(1.0, 0.0, 0.0)
v1 = chrono.ChVectorD(0.8, 0.0, 0.0) + displ
v2 = chrono.ChVectorD(0.8, 0.3, 0.0) + displ
v3 = chrono.ChVectorD(0.8, 0.3, 0.3) + displ
v4 = chrono.ChVectorD(0.0, 0.3, 0.3) + displ
v5 = chrono.ChVectorD(0.0, 0.0, 0.3) + displ
v6 = chrono.ChVectorD(0.8, 0.0, 0.3) + displ
mpoints = chrono.vector_ChVectorD([v1 , v2, v3, v4, v5, v6])

mhull = chrono.ChBodyEasyConvexHullAuxRef(mpoints, 1000, True, True, chrono.ChMaterialSurfaceNSC())
 
mhull.Move(chrono.ChVectorD(2, 0.3, 0))
mphysicalSystem.Add(mhull)


#####################################

# ==IMPORTANT== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in 
# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

application.AssetBindAll()

# ==IMPORTANT== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

application.AssetUpdateAll()

#
# THE SOFT-REAL-TIME CYCLE
#
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while application.GetDevice().run():
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

