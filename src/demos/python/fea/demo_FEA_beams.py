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


import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print ("Example: PyChrono using  beam finite elements");

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')


# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#


# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

## Create a mesh, that is a container for groups
## of elements and their referenced nodes.
mesh = fea.ChMesh();

## Create a section, i.e. thickness and material properties
## for beams. This will be shared among some beams.

msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetShearModulus(0.01e9 * 0.3)
msection.SetRayleighDamping(0.000)
#msection.SetCentroid(0,0.02)
#msection.SetShearCenter(0,0.1)
#msection.SetSectionRotation(45*chrono.CH_RAD_TO_DEG)

# Add some EULER-BERNOULLI BEAMS:

beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(beam_L * 2, 0, 0)))

mesh.AddNode(hnode1)
mesh.AddNode(hnode2)
mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()

belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)

mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()

belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)

mesh.AddElement(belement2);

# Apply a force or a torque to a node:
hnode2.SetForce(chrono.ChVector3d(4, 2, 0))
hnode3.SetTorque(chrono.ChVector3d(0, -0.04, 0))

# Fix a node to ground:
#    hnode1.SetFixed(True)
# otherwise fix it using constraints:

mtruss = chrono.ChBody()
mtruss.SetFixed(True)
sys.Add(mtruss)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
sys.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,   # x, y, z
                               True, True, True)   # Rx, Ry, Rz

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
sys.Add(constr_d)
constr_d.SetConstrainedCoords(False, True, True,     # x, y, z
                              False, False,False)    # Rx, Ry, Rz


# Add some EULER-BERNOULLI BEAMS (the fast way!)

# Shortcut!
# This ChBuilderBeamEuler helper object is very useful because it will
# subdivide 'beams' into sequences of finite elements of beam type, ex.
# one 'beam' could be made of 5 FEM elements of ChElementBeamEuler class.
# If new nodes are needed, it will create them for you.

builder = fea.ChBuilderBeamEuler()

# Now, simply use BuildBeam to create a beam from a point to another:
builder.BuildBeam(mesh,                   # the mesh where to put the created nodes and elements
                    msection,                  # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                    5,                         # the number of ChElementBeamEuler to create
                    chrono.ChVector3d(0, 0, -0.1),   # the 'A' point in space (beginning of beam)
                    chrono.ChVector3d(0.2, 0, -0.1), # the 'B' point in space (end of beam)
                    chrono.ChVector3d(0, 1, 0))      # the 'Y' up direction of the section for the beam

## After having used BuildBeam(), you can retrieve the nodes used for the beam,
## For example say you want to fix the A end and apply a force to the B end:
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVector3d(0, -1, 0))

# Again, use BuildBeam for creating another beam, this time
# it uses one node (the last node created by the last beam) and one point:
builder.BuildBeam(mesh, msection, 5,
                    builder.GetLastBeamNodes().front(),  # the 'A' node in space (beginning of beam)
                    chrono.ChVector3d(0.2, 0.1, -0.1),    # the 'B' point in space (end of beam)
                    chrono.ChVector3d(0, 1, 0));          # the 'Y' up direction of the section for the beam


# We do not want gravity effect on FEA elements in this demo
mesh.SetAutomaticGravity(False);

# Remember to add the mesh to the system!
sys.Add(mesh)

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.

visualizebeamA = chrono.ChVisualShapeFEA(mesh)
visualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
visualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
visualizebeamA.SetSmoothFaces(True)
visualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(visualizebeamA)

visualizebeamC = chrono.ChVisualShapeFEA(mesh)
visualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
visualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizebeamC.SetSymbolsThickness(0.006)
visualizebeamC.SetSymbolsScale(0.01)
visualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.1, 0.1, 0.2))
vis.AddTypicalLights()


# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverPardisoMKL()
sys.SetSolver(msolver)


# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)






