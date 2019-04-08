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



print ("Example: create a rigid body based on a .obj mesh file");

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chrono.SetChronoDataPath("../../../data/")

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystemNSC()



# Set the global collision margins. This is expecially important for very large or
# very small objects. Set this before creating shapes. Not before creating mysystem.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001);
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001);

# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

# Create a floor
mfloor = chrono.ChBodyEasyBox(3, 0.2, 3, 1000,True,True)
mfloor.SetBodyFixed(True)
mysystem.Add(mfloor)


# Now we will create a falling object whose shape is defined by a .obj mesh.
#
# NOTE: collision detection with generic concave meshes is slower and less
# robust than any other options for collision shapes, so use it if defining 
# collision shapes via primitives like spheres boxes cylinders or their
# clusters is too complex.
#
# NOTE: the mesh shape is a .obj file in Wavefront file format,
# you can generate it from 3D modelers such as Blender, Maya, etc. 
#
# NOTE: for collision purposes, the .obj mesh must be "watertight", i.e. having
# no gaps in edges, no repeated vertexes, etc. 
#
# NOTE: for visualization purposes only, i.e. if you do not use the mesh also for 
# collision, the mesh does not need to be watertight. 


# Method A: 
# - use the ChBodyEasyMesh
# This will automatically create the visualization mesh, the collision mesh,
# and will automatically compute the mass property (COG position respect to REF, 
# mass and inertia tensor) given an uniform density.

body_A= chrono.ChBodyEasyMesh(chrono.GetChronoDataPath() +'shoe_view.obj', # mesh filename
                              7000, # density kg/m^3
                              True, # use mesh for visualization?
                              True) # use mesh for collision?
body_A.SetPos(chrono.ChVectorD(0.5,0.5,0))
mysystem.Add(body_A)



# Method B: 
# - create a ChBodyAuxRef, 
# - set mass and inertia tensor as you like
# - set COG center of mass position respect to REF reference as you like
# - attach a visualization shape based on a .obj triangle mesh
# - add contact shape based on a .obj triangle mesh
# This is more complicate than method A, yet this can be still preferred if you
# need deeper control, ex. you want to provide two different meshes, one
# with high level of detail just for the visualization and a coarse one for
# collision, or if you want to set custom COG and inertia values, etc.

# Rigid body part
body_B= chrono.ChBodyAuxRef()
body_B.SetPos(chrono.ChVectorD(0,0.5,0))
body_B.SetMass(16)
body_B.SetInertiaXX(chrono.ChVectorD(0.270,0.400,0.427))
body_B.SetInertiaXY(chrono.ChVectorD(0.057,0.037,-0.062))
body_B.SetFrame_COG_to_REF(chrono.ChFrameD(
            chrono.ChVectorD( 0.12,0.0,0),
            chrono.ChQuaternionD(1,0,0,0)))

# Attach a visualization shape .
# First load a .obj from disk into a ChTriangleMeshConnected:
mesh_for_visualization = chrono.ChTriangleMeshConnected()
mesh_for_visualization.LoadWavefrontMesh(chrono.GetChronoDataPath() +'shoe_view.obj')
# Optionally: you can scale/shrink/rotate the mesh using this:
mesh_for_visualization.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
# Now the  triangle mesh is inserted in a ChTriangleMeshShape visualization asset, 
# and added to the body
visualization_shape = chrono.ChTriangleMeshShape()
visualization_shape.SetMesh(mesh_for_visualization)
body_B.AddAsset(visualization_shape)


# Add the collision shape.
# Again load a .obj file in Wavefront file format. NOTE: in this
# example we use the same .obj file as for visualization, but here one
# could do a better thing: using a different low-level-of-detail mesh for the 
# collision, so the simulation performance is not affected by many details such 
# as bolts and chamfers that may be wanted only for visualization.
mesh_for_collision = chrono.ChTriangleMeshConnected()
mesh_for_collision.LoadWavefrontMesh(chrono.GetChronoDataPath() +'shoe_view.obj')
# Optionally: you can scale/shrink/rotate the mesh using this:
mesh_for_collision.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
body_B.GetCollisionModel().ClearModel()
body_B.GetCollisionModel().AddTriangleMesh(
            mesh_for_collision, # the mesh 
            False,  # is it static?
            False)  # is it convex?
            # , mpos, mr,  # pos of mesh respect to REF and rotation matr.respect to REF 
            # 0.01) # 'inflating' radiust for triangles for increased robustness
body_B.GetCollisionModel().BuildModel()
body_B.SetCollide(True)

mysystem.Add(body_B)








# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
myapplication.AddTypicalCamera(chronoirr.vector3df(0.5,0.5,1), chronoirr.vector3df(0,0,0))
#myapplication.AddTypicalLights()
myapplication.AddLightWithShadow(chronoirr.vector3df(3,6,2),    # point
                                 chronoirr.vector3df(0,0,0),    # aimpoint
                                 12,                 # radius (power)
                                 1,11,              # near, far
                                 55)                # angle of FOV

            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			# If you need a finer control on which item really needs a visualization proxy in
			# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

myapplication.AssetBindAll();

			# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll();

            # If you want to show shadows because you used "AddLightWithShadow()'
            # you must remember this:
myapplication.AddShadowAll();


# ---------------------------------------------------------------------
#
#  Run the simulation
#


myapplication.SetTimestep(0.005)

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()


