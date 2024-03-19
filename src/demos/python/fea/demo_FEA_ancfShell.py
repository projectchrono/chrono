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
# Authors: Simone Benatti
# =============================================================================
#
# Demo on using ANCF shell elements
#
# =============================================================================


import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr
import math


#print(["Copyright (c) 2017 projectchrono.org\nChrono version: ", chrono.CHRONO_VERSION , "\n\n"])

def CastNode(nb):

    feaNB = fea.CastToChNodeFEAbase(nb)
    nodeFead = fea.CastToChNodeFEAxyzD(feaNB)
    return nodeFead


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

time_step = 1e-3

sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

print( "-----------------------------------------------------------\n")
print("------------------------------------------------------------\n")
print("     ANCF Shell Elements demo with implicit integration     \n")
print( "-----------------------------------------------------------\n")

# Create a mesh, that is a container for groups of elements and their referenced nodes.
mesh = fea.ChMesh()
numFlexBody = 1
# Geometry of the plate
plate_lenght_x = 1
plate_lenght_y = 0.1
plate_lenght_z = 0.01
# Specification of the mesh
numDiv_x = 10
numDiv_y = 2
N_x = numDiv_x + 1
N_y = numDiv_y + 1
# Number of elements in the z direction is considered as 1
TotalNumElements = numDiv_x * numDiv_y
TotalNumNodes = N_x * N_y
# For uniform mesh
dx = plate_lenght_x / numDiv_x
dy = plate_lenght_y / numDiv_y
dz = plate_lenght_z

# Create and add the nodes
for i in range(TotalNumNodes) :
    # Node location
    loc_x = (i % N_x) * dx;
    loc_y = (i // N_x) % N_y * dy;
    loc_z = 0;

    # Node direction
    dir_x = 0
    dir_y = 0
    dir_z = 1

    # Create the node
    node = fea.ChNodeFEAxyzD(chrono.ChVector3d(loc_x, loc_y, loc_z), chrono.ChVector3d(dir_x, dir_y, dir_z))

    node.SetMass(0)

    # Fix all nodes along the axis X=0
    if (i % (numDiv_x + 1) == 0):
        node.SetFixed(True)

    # Add node to mesh
    mesh.AddNode(node)


# Get a handle to the tip node.
tempnode = mesh.GetNode(TotalNumNodes - 1)
tempfeanode = fea.CastToChNodeFEAbase(tempnode)
nodetip = fea.CastToChNodeFEAxyzD(tempfeanode)

# Create an orthotropic material.
# All layers for all elements share the same material.
rho = 500
E = chrono.ChVector3d(2.1e7, 2.1e7, 2.1e7)
nu = chrono.ChVector3d(0.3, 0.3, 0.3)
G = chrono.ChVector3d(8.0769231e6, 8.0769231e6, 8.0769231e6)
mat = fea.ChMaterialShellANCF(rho, E, nu, G)
# Create the elements
for i in range(TotalNumElements):
    # Adjacent nodes
    node0 = (i // numDiv_x) * N_x + i % numDiv_x
    node1 = (i // numDiv_x) * N_x + i % numDiv_x + 1
    node2 = (i // numDiv_x) * N_x + i % numDiv_x + 1 + N_x
    node3 = (i // numDiv_x) * N_x + i % numDiv_x + N_x

    # Create the element and set its nodes.
    element = fea.ChElementShellANCF_3423()
    element.SetNodes(CastNode(mesh.GetNode(node0)),
                      CastNode(mesh.GetNode(node1)),
                      CastNode(mesh.GetNode(node2)),
                      CastNode(mesh.GetNode(node3)))

    # Set element dimensions
    element.SetDimensions(dx, dy)

    # Add a single layers with a fiber angle of 0 degrees.
    element.AddLayer(dz, 0 * chrono.CH_DEG_TO_RAD, mat)

    # Set other element properties
    element.SetAlphaDamp(0.0)    # Structural damping for this element
    
    # Add element to mesh
    mesh.AddElement(element)


# Add the mesh to the system
sys.Add(mesh)

# -------------------------------------
# Options for visualization in irrlicht
# -------------------------------------

visualizemeshA = chrono.ChVisualShapeFEA(mesh)
visualizemeshA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_SPEED_NORM)
visualizemeshA.SetColorscaleMinMax(0.0, 5.50)
visualizemeshA.SetShrinkElements(True, 0.85)
visualizemeshA.SetSmoothFaces(True)
mesh.AddVisualShapeFEA(visualizemeshA)

visualizemeshB = chrono.ChVisualShapeFEA(mesh)
visualizemeshB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
visualizemeshB.SetWireframe(True)
visualizemeshB.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(visualizemeshB)

visualizemeshC = chrono.ChVisualShapeFEA(mesh)
visualizemeshC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
visualizemeshC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizemeshC.SetSymbolsThickness(0.004)
mesh.AddVisualShapeFEA(visualizemeshC)

visualizemeshD = chrono.ChVisualShapeFEA(mesh)
visualizemeshD.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_ELEM_TENS_STRAIN)
visualizemeshD.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
visualizemeshD.SetSymbolsScale(1)
visualizemeshD.SetColorscaleMinMax(-0.5, 5)
visualizemeshD.SetZbufferHide(False)
mesh.AddVisualShapeFEA(visualizemeshD)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('ANCF shells')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-0.4, -1.3, 0.0), chrono.ChVector3d(0.0, 0.5, -0.1))
vis.AddTypicalLights()

# ----------------------------------
# Perform a dynamic time integration
# ----------------------------------

# Set up solver

solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)

solver.EnableDiagonalPreconditioner(True)
#solver.SetVerbose(True)

sys.GetSolver().AsIterative().SetMaxIterations(100)
sys.GetSolver().AsIterative().SetTolerance(1e-10)

# Set up integrator

stepper = chrono.ChTimestepperHHT(sys)
sys.SetTimestepper(stepper)

stepper.SetAlpha(-0.2)
stepper.SetMaxIters(5)
stepper.SetAbsTolerances(1e-2)
stepper.SetStepControl(True)
stepper.SetMinStepSize(1e-4)
#stepper.SetVerbose(True)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)

