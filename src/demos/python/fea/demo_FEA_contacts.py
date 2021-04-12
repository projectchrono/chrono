# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simone Benatti
# =============================================================================
#
# FEA contacts
#
# =============================================================================

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr



print("Copyright (c) 2017 projectchrono.org")

# Create a Chrono::Engine physical system
my_system = chrono.ChSystemSMC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(my_system, "FEA contacts", chronoirr.dimension2du(1024, 768))

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0, 0.6, -1))
application.AddLightWithShadow(chronoirr.vector3df(1.5, 5.5, -2.5), chronoirr.vector3df(0, 0, 0), 3, 2.2, 7.2, 40, 512,
                               chronoirr.SColorf(1, 1, 1))

application.SetContactsDrawMode(chronoirr.IrrContactsDrawMode_CONTACT_DISTANCES)

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

mysurfmaterial = chrono.ChMaterialSurfaceSMC()
mysurfmaterial.SetYoungModulus(6e4)
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
    mfloor.SetPos(chrono.ChVectorD(0, -1, 0))
    mfloor.SetBodyFixed(True)
    my_system.Add(mfloor)
    
    mfloor.GetCollisionModel().ClearModel()
    mfloor.GetCollisionModel().AddTriangleMesh(mysurfmaterial, mmeshbox, False, False, chrono.VNULL, chrono.ChMatrix33D(1),
                                     sphere_swept_thickness)
    mfloor.GetCollisionModel().BuildModel()
    mfloor.SetCollide(True)
    
    masset_meshbox = chrono.ChTriangleMeshShape()
    masset_meshbox.SetMesh(mmeshbox)
    mfloor.AddAsset(masset_meshbox)
    
    masset_texture = chrono.ChTexture()
    masset_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    mfloor.AddAsset(masset_texture)

else :
    # floor as a simple collision primitive:
    
    mfloor = chrono.ChBodyEasyBox(2, 0.1, 2, 2700, True, True, mysurfmaterial)
    mfloor.SetBodyFixed(True)
    my_system.Add(mfloor)
    
    masset_texture = chrono.ChTexture()
    masset_texture.SetTextureFilename(chrono.GetChronoDataFile("textures/concrete.jpg"))
    mfloor.AddAsset(masset_texture)


# two falling objects:

mcube = chrono.ChBodyEasyBox(0.1, 0.1, 0.1, 2700, True, True, mysurfmaterial)
mcube.SetPos(chrono.ChVectorD(0.6, 0.5, 0.6))
my_system.Add(mcube)

msphere = chrono.ChBodyEasySphere(0.1, 2700, True, True, mysurfmaterial)
msphere.SetPos(chrono.ChVectorD(0.8, 0.5, 0.6))
my_system.Add(msphere)

#
# Example 1: tetrahedrons, with collisions
#

# Create a mesh. We will use it for tetrahedrons.

my_mesh = fea.ChMesh()

# 1) a FEA tetrahedron(s):

# Create a material, that must be assigned to each solid element in the mesh,
# and set its parameters
mmaterial = fea.ChContinuumElastic()
mmaterial.Set_E(0.01e9)  # rubber 0.01e9, steel 200e9
mmaterial.Set_v(0.3)
mmaterial.Set_RayleighDampingK(0.003)
mmaterial.Set_density(1000)


for i in range(4) :
    try :
        cdown = chrono.ChCoordsysD(chrono.ChVectorD(0, -0.4, 0))
        crot = chrono.ChCoordsysD(chrono.VNULL, chrono.Q_from_AngAxis(chrono.CH_C_2PI * chrono.ChRandom(), 
                                                                     chrono.VECT_Y) * chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X))
        cydisp = chrono.ChCoordsysD(chrono.ChVectorD(-0.3, 0.1 + i * 0.1, -0.3))
        ctot = cydisp.TransformLocalToParent(crot.TransformLocalToParent(cdown))
        mrot = chrono.ChMatrix33D(ctot.rot)
        fea.ChMeshFileLoader.FromTetGenFile(my_mesh, chrono.GetChronoDataFile("fea/beam.node"),
                                     chrono.GetChronoDataFile("fea/beam.ele"), mmaterial, ctot.pos, mrot)
    except :
        print('Error Loading meshes')
        break
    


# Create the contact surface(s).
# In this case it is a ChContactSurfaceMesh, that allows mesh-mesh collsions.

mcontactsurf = fea.ChContactSurfaceMesh(mysurfmaterial)
my_mesh.AddContactSurface(mcontactsurf)

mcontactsurf.AddFacesFromBoundary(sphere_swept_thickness)  # do this after my_meshAddContactSurface


# Remember to add the mesh to the system!
my_system.Add(my_mesh)

#
# Example 2: beams, with collisions
#

# Create a mesh. We will use it for beams only.

my_mesh_beams = fea.ChMesh()

# 2) an ANCF cable:

msection_cable2 = fea.ChBeamSectionCable()
msection_cable2.SetDiameter(0.05)
msection_cable2.SetYoungModulus(0.01e9)
msection_cable2.SetBeamRaleyghDamping(0.05)

builder = fea.ChBuilderCableANCF()

builder.BuildBeam(my_mesh_beams,             # the mesh where to put the created nodes and elements
  msection_cable2,           # the ChBeamSectionCable to use for the ChElementCableANCF elements
  10,                        # the number of ChElementCableANCF to create
  chrono.ChVectorD(0, 0.1, -0.1),  # the 'A' poin space (beginning of beam)
  chrono.ChVectorD(0.5, 0.13, -0.1))  # the 'B' poin space (end of beam)

# Create the contact surface(s).
# In this case it is a ChContactSurfaceNodeCloud, so just pass
# all nodes to it.

mcontactcloud = fea.ChContactSurfaceNodeCloud(mysurfmaterial)
my_mesh_beams.AddContactSurface(mcontactcloud)

mcontactcloud.AddAllNodes(0.025)  # use larger posize to match beam section radius


# Remember to add the mesh to the system!
my_system.Add(my_mesh_beams)

#
# Optional...  visualization
#

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChTriangleMeshShape).
# Do not forget AddAsset() at the end!

mvisualizemesh = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
mvisualizemesh.SetColorscaleMinMax(0.0, 5.50)
mvisualizemesh.SetSmoothFaces(True)
my_mesh.AddAsset(mvisualizemesh)

mvisualizemeshcoll = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshcoll.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_CONTACTSURFACES)
mvisualizemeshcoll.SetWireframe(True)
mvisualizemeshcoll.SetDefaultMeshColor(chrono.ChColor(1, 0.5, 0))
my_mesh.AddAsset(mvisualizemeshcoll)

mvisualizemeshbeam = fea.ChVisualizationFEAmesh(my_mesh_beams)
mvisualizemeshbeam.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
mvisualizemeshbeam.SetColorscaleMinMax(0.0, 5.50)
mvisualizemeshbeam.SetSmoothFaces(True)
my_mesh.AddAsset(mvisualizemeshbeam)

mvisualizemeshbeamnodes = fea.ChVisualizationFEAmesh(my_mesh_beams)
mvisualizemeshbeamnodes.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizemeshbeamnodes.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizemeshbeamnodes.SetSymbolsThickness(0.008)
my_mesh.AddAsset(mvisualizemeshbeamnodes)

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.

application.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

application.AssetUpdateAll()

# Use shadows in realtime view
application.AddShadowAll()

# SIMULATION LOOP

solver = chrono.ChSolverMINRES()
my_system.SetSolver(solver)
solver.SetMaxIterations(40)
solver.SetTolerance(1e-12)
solver.EnableDiagonalPreconditioner(True)
solver.EnableWarmStart(True)  # Enable for better convergence when using Euler implicit linearized

my_system.SetSolverForceTolerance(1e-10)

application.SetTimestep(0.001)

while application.GetDevice().run() :
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

