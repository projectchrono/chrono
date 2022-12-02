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



print ("Example: create a rigid body based on a .obj mesh file");

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

sys      = chrono.ChSystemNSC()



# Set the global collision margins. This is expecially important for very large or
# very small objects. Set this before creating shapes. Not before creating sys.
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001);
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001);

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

# Create a contact material (with default properties, shared by all collision shapes)
contact_material = chrono.ChMaterialSurfaceNSC()

# Create a floor
mfloor = chrono.ChBodyEasyBox(3, 0.2, 3, 1000,True,True, contact_material)
mfloor.SetBodyFixed(True)
mfloor.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.2, 0.6))
sys.Add(mfloor)


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

body_A= chrono.ChBodyEasyMesh(chrono.GetChronoDataFile('models/bulldozer/shoe_view.obj'), # mesh filename
                              7000,             # density kg/m^3
                              True,             # automatically compute mass and inertia
                              True,             # visualize?>
                              True,             # collide?
                              contact_material, # contact material
                              )
body_A.SetPos(chrono.ChVectorD(0.5,0.5,0))
body_A.GetVisualShape(0).SetColor(chrono.ChColor(0.2, 0.6, 0.2))
sys.Add(body_A)



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
mesh_for_visualization.LoadWavefrontMesh(chrono.GetChronoDataFile('models/bulldozer/shoe_view.obj'))
# Optionally: you can scale/shrink/rotate the mesh using this:
mesh_for_visualization.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
# Now the  triangle mesh is inserted in a ChTriangleMeshShape visualization asset, 
# and added to the body
visualization_shape = chrono.ChTriangleMeshShape()
visualization_shape.SetMesh(mesh_for_visualization)
visualization_shape.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
body_B.AddVisualShape(visualization_shape)


# Add the collision shape.
# Again load a .obj file in Wavefront file format. NOTE: in this
# example we use the same .obj file as for visualization, but here one
# could do a better thing: using a different low-level-of-detail mesh for the 
# collision, so the simulation performance is not affected by many details such 
# as bolts and chamfers that may be wanted only for visualization.
mesh_for_collision = chrono.ChTriangleMeshConnected()
mesh_for_collision.LoadWavefrontMesh(chrono.GetChronoDataFile('models/bulldozer/shoe_view.obj'))
# Optionally: you can scale/shrink/rotate the mesh using this:
mesh_for_collision.Transform(chrono.ChVectorD(0.01,0,0), chrono.ChMatrix33D(1))
body_B.GetCollisionModel().ClearModel()
body_B.GetCollisionModel().AddTriangleMesh(
            contact_material, # contact material
            mesh_for_collision, # the mesh 
            False,  # is it static?
            False)  # is it convex?
            # , mpos, mr,  # pos of mesh respect to REF and rotation matr.respect to REF 
            # 0.01) # 'inflating' radiust for triangles for increased robustness
body_B.GetCollisionModel().BuildModel()
body_B.SetCollide(True)

sys.Add(body_B)

#  Create an Irrlicht application to visualize the sys
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Trimesh collision demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.5,0.5,1))
vis.AddTypicalLights()

#  Run the simulation
while vis.Run():
    vis.BeginScene() 
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(5e-3)


