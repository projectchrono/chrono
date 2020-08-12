#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Alessandro Tasora
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


import pychrono as chrono
import pychrono.fea as fea
import pychrono.mkl as mkl
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
my_system = chrono.ChSystemSMC()

## Create a mesh, that is a container for groups
## of elements and their referenced nodes.
my_mesh = fea.ChMesh();

## Create a section, i.e. thickness and material properties
## for beams. This will be shared among some beams.

msection = fea.ChBeamSectionEulerAdvanced()

beam_wy = 0.012
beam_wz = 0.025
msection.SetAsRectangularSection(beam_wy, beam_wz)
msection.SetYoungModulus(0.01e9)
msection.SetGshearModulus(0.01e9 * 0.3)
msection.SetBeamRaleyghDamping(0.000)
#msection.SetCentroid(0,0.02)
#msection.SetShearCenter(0,0.1)
#msection.SetSectionRotation(45*chrono.CH_C_RAD_TO_DEG)

# Add some EULER-BERNOULLI BEAMS:

beam_L = 0.1

hnode1 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(0, 0, 0)))
hnode2 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L, 0, 0)))
hnode3 = fea.ChNodeFEAxyzrot(chrono.ChFrameD(chrono.ChVectorD(beam_L * 2, 0, 0)))

my_mesh.AddNode(hnode1)
my_mesh.AddNode(hnode2)
my_mesh.AddNode(hnode3)

belement1 = fea.ChElementBeamEuler()

belement1.SetNodes(hnode1, hnode2)
belement1.SetSection(msection)

my_mesh.AddElement(belement1)

belement2 = fea.ChElementBeamEuler()

belement2.SetNodes(hnode2, hnode3)
belement2.SetSection(msection)

my_mesh.AddElement(belement2);

# Apply a force or a torque to a node:
hnode2.SetForce(chrono.ChVectorD(4, 2, 0))
hnode3.SetTorque(chrono.ChVectorD(0, -0.04, 0))

# Fix a node to ground:
#    hnode1.SetFixed(True)
# otherwise fix it using constraints:

mtruss = chrono.ChBody()
mtruss.SetBodyFixed(True)
my_system.Add(mtruss)

constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(hnode3, mtruss, False, hnode3.Frame(), hnode3.Frame())
my_system.Add(constr_bc)
constr_bc.SetConstrainedCoords(True, True, True,   # x, y, z
                               True, True, True)   # Rx, Ry, Rz

constr_d = chrono.ChLinkMateGeneric()
constr_d.Initialize(hnode1, mtruss, False, hnode1.Frame(), hnode1.Frame())
my_system.Add(constr_d)
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
builder.BuildBeam(my_mesh,                   # the mesh where to put the created nodes and elements
                    msection,                  # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                    5,                         # the number of ChElementBeamEuler to create
                    chrono.ChVectorD(0, 0, -0.1),   # the 'A' point in space (beginning of beam)
                    chrono.ChVectorD(0.2, 0, -0.1), # the 'B' point in space (end of beam)
                    chrono.ChVectorD(0, 1, 0))      # the 'Y' up direction of the section for the beam

## After having used BuildBeam(), you can retrieve the nodes used for the beam,
## For example say you want to fix the A end and apply a force to the B end:
builder.GetLastBeamNodes().back().SetFixed(True)
builder.GetLastBeamNodes().front().SetForce(chrono.ChVectorD(0, -1, 0))

# Again, use BuildBeam for creating another beam, this time
# it uses one node (the last node created by the last beam) and one point:
builder.BuildBeam(my_mesh, msection, 5,
                    builder.GetLastBeamNodes().front(),  # the 'A' node in space (beginning of beam)
                    chrono.ChVectorD(0.2, 0.1, -0.1),    # the 'B' point in space (end of beam)
                    chrono.ChVectorD(0, 1, 0));          # the 'Y' up direction of the section for the beam


# We do not want gravity effect on FEA elements in this demo
my_mesh.SetAutomaticGravity(False);

# Remember to add the mesh to the system!
my_system.Add(my_mesh)

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.

mvisualizebeamA = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
mvisualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
my_mesh.AddAsset(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizebeamC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizebeamC)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#


# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
myapplication = chronoirr.ChIrrApp(my_system, 'Test FEA beams', chronoirr.dimension2du(1024,768))

#application.AddTypicalLogo()
myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(0.1,0.1,0.2))
myapplication.AddTypicalLights()

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.

myapplication.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll()


# THE SOFT-REAL-TIME CYCLE


# Change the solver form the default SOR to the MKL Pardiso, more precise for fea.
msolver = mkl.ChSolverMKL()
my_system.SetSolver(msolver)


myapplication.SetTimestep(0.001);

#	application.GetSystem().DoStaticLinear()

while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()






