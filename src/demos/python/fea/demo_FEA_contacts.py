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

# Create a Chrono::Engine physical system
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

mysurfmaterial = chrono.ChContactMaterialSMC()
mysurfmaterial.SetYoungModulus(1e5)
mysurfmaterial.SetFriction(0.3)
mysurfmaterial.SetRestitution(0.2)
mysurfmaterial.SetAdhesion(0)

# Create a floor:

do_mesh_collision_floor = False

mmeshbox = chrono.ChTriangleMeshConnected()
mmeshbox.LoadWavefrontMesh(chrono.GetChronoDataFile("models/cube.obj"), True, True)

if (do_mesh_collision_floor) :
    # floor as a triangle mesh surface:
    mfloor = chrono.chronoChBody()
    mfloor.SetPos(chrono.ChVector3d(0, -1, 0))
    mfloor.SetFixed(True)
    sys.Add(mfloor)
    
    mfloor.GetCollisionModel().Clear()
    mfloor_ct_shape = chrono.ChCollisionShapeTriangleMesh(mysurfmaterial, mmeshbox, False, False, sphere_swept_thickness)
    mfloor.GetCollisionModel().AddShape(mfloor_ct_shape)
    mfloor.GetCollisionModel().Build()
    mfloor.EnableCollision(True)
    
    masset_meshbox = chrono.ChVisualShapeTriangleMesh()
    masset_meshbox.SetMesh(mmeshbox)
    mfloor.AddVisualShape(masset_meshbox)
    
    masset_texture = chrono.ChTexture()
    masset_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    mfloor.AddVisualShapeFEA(masset_texture)

else :
    # floor as a simple collision primitive:
    
    mfloor = chrono.ChBodyEasyBox(2, 0.1, 2, 2700, True, True, mysurfmaterial)
    mfloor.SetFixed(True)
    mfloor.GetVisualShape(0).SetTexture(chrono.GetChronoDataFile("textures/concrete.jpg"))
    sys.Add(mfloor)


# two falling objects:

mcube = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 2700, True, True, mysurfmaterial)
mcube.SetPos(chrono.ChVector3d(0.6, 0.5, 0.6))
sys.Add(mcube)

msphere = chrono.ChBodyEasySphere(0.1, 2700, True, True, mysurfmaterial)
msphere.SetPos(chrono.ChVector3d(0.8, 0.5, 0.6))
sys.Add(msphere)

#
# Example 1: tetrahedrons, with collisions
#

# Create a mesh. We will use it for tetrahedrons.

mesh = fea.ChMesh()

# 1) a FEA tetrahedron(s):

# Create a material, that must be assigned to each solid element in the mesh,
# and set its parameters
mmaterial = fea.ChContinuumElastic()
mmaterial.SetYoungModulus(0.01e9)  # rubber 0.01e9, steel 200e9
mmaterial.SetPoissonRatio(0.3)
mmaterial.SetRayleighDampingBeta(0.003)
mmaterial.SetDensity(1000)

for i in range(4) :
    try :
        cdown = chrono.ChCoordsysd(chrono.ChVector3d(0, -0.4, 0))
        crot = chrono.ChCoordsysd(chrono.VNULL, chrono.QuatFromAngleAxis(chrono.CH_2PI * chrono.ChRandom.Get(), chrono.VECT_Y) * 
                                                chrono.QuatFromAngleAxis(chrono.CH_PI_2, chrono.VECT_X))
        cydisp = chrono.ChCoordsysd(chrono.ChVector3d(-0.3, 0.1 + i * 0.1, -0.3))
        ctot = cydisp.TransformLocalToParent(crot.TransformLocalToParent(cdown))
        mrot = chrono.ChMatrix33d(ctot.rot)
        fea.ChMeshFileLoader.FromTetGenFile(mesh, chrono.GetChronoDataFile("fea/beam.node"),
                                     chrono.GetChronoDataFile("fea/beam.ele"), mmaterial, ctot.pos, mrot)
    except :
        print('Error Loading meshes')
        break
    
# Create the contact surface(s).
# In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collsions.
mcontactsurf = fea.ChContactSurfaceMesh(mysurfmaterial)
mcontactsurf.AddFacesFromBoundary(mesh, sphere_swept_thickness)
mesh.AddContactSurface(mcontactsurf)

# Add the mesh to the system
sys.Add(mesh)

#
# Example 2: beams, with collisions
#

# Create a mesh. We will use it for beams only.

my_mesh_beams = fea.ChMesh()

# 2) an ANCF cable:

msection_cable2 = fea.ChBeamSectionCable()
msection_cable2.SetDiameter(0.05)
msection_cable2.SetYoungModulus(0.01e9)
msection_cable2.SetRayleighDamping(0.05)

builder = fea.ChBuilderCableANCF()

builder.BuildBeam(my_mesh_beams,             # the mesh where to put the created nodes and elements
  msection_cable2,           # the ChBeamSectionCable to use for the ChElementCableANCF elements
  10,                        # the number of ChElementCableANCF to create
  chrono.ChVector3d(0, 0.1, -0.1),  # the 'A' poin space (beginning of beam)
  chrono.ChVector3d(0.5, 0.13, -0.1))  # the 'B' poin space (end of beam)

# Create the contact surface(s).
# In this case it is a ChContactSurfaceNodeCloud, so just pass
# all nodes to it.
mcontactcloud = fea.ChContactSurfaceNodeCloud(mysurfmaterial)
mcontactcloud.AddAllNodes(my_mesh_beams, 0.025)  # match beam section radius
my_mesh_beams.AddContactSurface(mcontactcloud)

# Add the mesh to the system
sys.Add(my_mesh_beams)

#
# Optional...  visualization
#

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChVisualShapeTriangleMesh).

mvisualizemesh = chrono.ChVisualShapeFEA()
mvisualizemesh.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mvisualizemesh.SetColorscaleMinMax(0.0, 5.50)
mvisualizemesh.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(mvisualizemesh)

mvisualizemeshcoll = chrono.ChVisualShapeFEA()
mvisualizemeshcoll.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_CONTACTSURFACES)
mvisualizemeshcoll.SetWireframe(True)
mvisualizemeshcoll.SetDefaultMeshColor(chrono.ChColor(1, 0.5, 0))
mesh.AddVisualShapeFEA(mvisualizemeshcoll)

mvisualizemeshbeam = chrono.ChVisualShapeFEA()
mvisualizemeshbeam.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
mvisualizemeshbeam.SetColorscaleMinMax(0.0, 5.50)
mvisualizemeshbeam.SetSmoothFaces(True)
my_mesh_beams.AddVisualShapeFEA(mvisualizemeshbeam)

mvisualizemeshbeamnodes = chrono.ChVisualShapeFEA()
mvisualizemeshbeamnodes.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizemeshbeamnodes.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizemeshbeamnodes.SetSymbolsThickness(0.008)
my_mesh_beams.AddVisualShapeFEA(mvisualizemeshbeamnodes)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA contacts')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0.6, -1))
vis.AddTypicalLights()

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

