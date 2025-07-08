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

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr



print("Copyright (c) 2017 projectchrono.org")

# Create a Chrono physical system
sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

#
# CREATE THE PHYSICAL SYSTEM
#

# Set default effective radius of curvature for all SCM contacts.
chrono.ChCollisionInfo.SetDefaultEffectiveCurvatureRadius(1)

# collision::ChCollisionModel::SetDefaultSuggestedEnvelope(0.0) # not needed, already 0 when using ChSystemSMC
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.006)  # max inside penetration - if not enough stiffness in material: troubles

# Use this value for an outward additional layer around meshes, that can improve
# robustness of mesh-mesh collision detection (at the cost of having unnatural inflate effect)
sphere_swept_thickness = 0.002

# Create the surface material, containing information
# about friction etc.
# It is a SMC (penalty) material that we will assign to
# all surfaces that might generate contacts.

contact_material = chrono.ChContactMaterialSMC()
contact_material.SetYoungModulus(1e5)
contact_material.SetFriction(0.3)
contact_material.SetRestitution(0.2)
contact_material.SetAdhesion(0)

# Create a floor:

do_mesh_collision_floor = False

box_mesh = chrono.ChTriangleMeshConnected()
box_mesh.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"), True, True)

if (do_mesh_collision_floor) :
    # floor as a triangle mesh surface:
    floor = chrono.chronoChBody()
    floor.SetPos(chrono.ChVector3d(0, -1, 0))
    floor.SetFixed(True)
    sys.Add(floor)
    
    floor.GetCollisionModel().Clear()
    mfloor_ct_shape = chrono.ChCollisionShapeTriangleMesh(contact_material, box_mesh, False, False, sphere_swept_thickness)
    floor.GetCollisionModel().AddShape(mfloor_ct_shape)
    floor.GetCollisionModel().Build()
    floor.EnableCollision(True)
    
    box_mesh_shape = chrono.ChVisualShapeTriangleMesh()
    box_mesh_shape.SetMesh(box_mesh)
    floor.AddVisualShape(box_mesh_shape)
    
    box_texture = chrono.ChTexture()
    box_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    floor.AddVisualShapeFEA(box_texture)

else :
    # floor as a simple collision primitive:
    
    floor = chrono.ChBodyEasyBox(2, 0.1, 2, 2700, True, True, contact_material)
    floor.SetFixed(True)
    floor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(floor)


# two falling objects:

cube = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 2700, True, True, contact_material)
cube.SetPos(chrono.ChVector3d(0.6, 0.5, 0.6))
sys.Add(cube)

sphere = chrono.ChBodyEasySphere(0.1, 2700, True, True, contact_material)
sphere.SetPos(chrono.ChVector3d(0.8, 0.5, 0.6))
sys.Add(sphere)

#
# Example 1: tetrahedrons, with collisions
#

# Create a mesh. We will use it for tetrahedrons.

mesh = fea.ChMesh()

# 1) a FEA tetrahedron(s):

# Create a material, that must be assigned to each solid element in the mesh,
# and set its parameters
material = fea.ChContinuumElastic()
material.SetYoungModulus(0.01e9)  # rubber 0.01e9, steel 200e9
material.SetPoissonRatio(0.3)
material.SetRayleighDampingBeta(0.003)
material.SetDensity(1000)

for i in range(4) :
    try :
        cdown = chrono.ChCoordsysd(chrono.ChVector3d(0, -0.4, 0))
        crot = chrono.ChCoordsysd(chrono.VNULL, chrono.QuatFromAngleY(chrono.CH_2PI * chrono.ChRandom.Get()) * chrono.QuatFromAngleX(chrono.CH_PI_2))
        cydisp = chrono.ChCoordsysd(chrono.ChVector3d(-0.3, 0.1 + i * 0.1, -0.3))
        ctot = cydisp.TransformLocalToParent(crot.TransformLocalToParent(cdown))
        rot = chrono.ChMatrix33d(ctot.rot)
        fea.ChMeshFileLoader.FromTetGenFile(mesh, chrono.GetChronoDataFile("fea/beam.node"),
                                     chrono.GetChronoDataFile("fea/beam.ele"), material, ctot.pos, rot)
    except :
        print('Error Loading meshes')
        break
    
# Create the contact surface(s).
# In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collsions.
contact_surface = fea.ChContactSurfaceMesh(contact_material)
contact_surface.AddFacesFromBoundary(mesh, sphere_swept_thickness)
mesh.AddContactSurface(contact_surface)

# Add the mesh to the system
sys.Add(mesh)

#
# Example 2: beams, with collisions
#

# Create a mesh. We will use it for beams only.
mesh_beam = fea.ChMesh()

# 2) an ANCF cable:

section_cable = fea.ChBeamSectionCable()
section_cable.SetDiameter(0.05)
section_cable.SetYoungModulus(0.01e9)
section_cable.SetRayleighDamping(0.05)

builder = fea.ChBuilderCableANCF()

builder.BuildBeam(mesh_beam,           # the mesh where to put the created nodes and elements
  section_cable,                       # the ChBeamSectionCable to use for the ChElementCableANCF elements
  10,                                  # the number of ChElementCableANCF to create
  chrono.ChVector3d(0, 0.1, -0.1),     # the 'A' poin space (beginning of beam)
  chrono.ChVector3d(0.5, 0.13, -0.1))  # the 'B' poin space (end of beam)

# Create the contact surface(s).
# In this case it is a ChContactSurfaceNodeCloud, so just pass all nodes to it.
contact_nodecloud = fea.ChContactSurfaceNodeCloud(contact_material)
contact_nodecloud.AddAllNodes(mesh_beam, 0.025)  # match beam section radius
mesh_beam.AddContactSurface(contact_nodecloud)

# Add the mesh to the system
sys.Add(mesh_beam)

#
# Optional...  visualization
#

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChVisualShapeTriangleMesh).

colormap_type = chrono.ChColormap.Type_FAST
colormap_range = chrono.ChVector2d(0.0, 2.5)

vis_mesh_A = chrono.ChVisualShapeFEA()
vis_mesh_A.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_mesh_A.SetColormapRange(colormap_range)
vis_mesh_A.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(vis_mesh_A)

vis_mesh_B = chrono.ChVisualShapeFEA()
vis_mesh_B.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_CONTACTSURFACES)
vis_mesh_B.SetWireframe(True)
vis_mesh_B.SetDefaultMeshColor(chrono.ChColor(1, 0.5, 0))
mesh.AddVisualShapeFEA(vis_mesh_B)

vis_beam_A = chrono.ChVisualShapeFEA()
vis_beam_A.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
vis_beam_A.SetColormapRange(colormap_range)
vis_beam_A.SetSmoothFaces(True)
mesh_beam.AddVisualShapeFEA(vis_beam_A)

vis_beam_B = chrono.ChVisualShapeFEA()
vis_beam_B.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
vis_beam_B.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_B.SetSymbolsThickness(0.008)
mesh_beam.AddVisualShapeFEA(vis_beam_B)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA contacts')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()
vis.AddGuiColorbar('Mz (Nm)', colormap_range, colormap_type, False, chrono.ChVector2i(10, 100))
vis.EnableContactDrawing(chronoirr.ContactsDrawMode_CONTACT_DISTANCES)

# SIMULATION LOOP

solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(40)
solver.SetTolerance(1e-12)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)  # Enable for better convergence when using Euler implicit linearized

sys.GetSolver().AsIterative().SetTolerance(1e-10)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.0005)

