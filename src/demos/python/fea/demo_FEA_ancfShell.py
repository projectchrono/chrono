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

my_system = chrono.ChSystemSMC()
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.8))

# Create the Irrlicht visualization (open the Irrlicht device, bind a simple user interface, etc.)
application = chronoirr.ChIrrApp(my_system, "ANCF Shells", chronoirr.dimension2du(800, 600), False, True)

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(-0.4, -0.3, 0.0),  # camera location
                             chronoirr.vector3df(0.0, 0.5, -0.1))  # "look at" location

print( "-----------------------------------------------------------\n")
print("------------------------------------------------------------\n")
print("     ANCF Shell Elements demo with implicit integration     \n")
print( "-----------------------------------------------------------\n")

# Create a mesh, that is a container for groups of elements and their referenced nodes.
my_mesh = fea.ChMesh()
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
    node = fea.ChNodeFEAxyzD(chrono.ChVectorD(loc_x, loc_y, loc_z), chrono.ChVectorD(dir_x, dir_y, dir_z))

    node.SetMass(0)

    # Fix all nodes along the axis X=0
    if (i % (numDiv_x + 1) == 0):
        node.SetFixed(True)

    # Add node to mesh
    my_mesh.AddNode(node)


# Get a handle to the tip node.
tempnode = my_mesh.GetNode(TotalNumNodes - 1)
tempfeanode = fea.CastToChNodeFEAbase(tempnode)
nodetip = fea.CastToChNodeFEAxyzD(tempfeanode)

# Create an orthotropic material.
# All layers for all elements share the same material.
rho = 500
E = chrono.ChVectorD(2.1e7, 2.1e7, 2.1e7)
nu = chrono.ChVectorD(0.3, 0.3, 0.3)
G = chrono.ChVectorD(8.0769231e6, 8.0769231e6, 8.0769231e6)
mat = fea.ChMaterialShellANCF(rho, E, nu, G)
# Create the elements
for i in range(TotalNumElements):
    # Adjacent nodes
    node0 = (i // numDiv_x) * N_x + i % numDiv_x
    node1 = (i // numDiv_x) * N_x + i % numDiv_x + 1
    node2 = (i // numDiv_x) * N_x + i % numDiv_x + 1 + N_x
    node3 = (i // numDiv_x) * N_x + i % numDiv_x + N_x

    # Create the element and set its nodes.
    element = fea.ChElementShellANCF()
    element.SetNodes(CastNode(my_mesh.GetNode(node0)),
                      CastNode(my_mesh.GetNode(node1)),
                      CastNode(my_mesh.GetNode(node2)),
                      CastNode(my_mesh.GetNode(node3)))

    # Set element dimensions
    element.SetDimensions(dx, dy)

    # Add a single layers with a fiber angle of 0 degrees.
    element.AddLayer(dz, 0 * chrono.CH_C_DEG_TO_RAD, mat)

    # Set other element properties
    element.SetAlphaDamp(0.0)    # Structural damping for this element
    element.SetGravityOn(False)  # turn internal gravitational force calculation off
    
    # Add element to mesh
    my_mesh.AddElement(element)


# Add the mesh to the system
my_system.Add(my_mesh)

# -------------------------------------
# Options for visualization in irrlicht
# -------------------------------------

visualizemeshA = fea.ChVisualizationFEAmesh(my_mesh)
visualizemeshA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
visualizemeshA.SetColorscaleMinMax(0.0, 5.50)
visualizemeshA.SetShrinkElements(True, 0.85)
visualizemeshA.SetSmoothFaces(True)
my_mesh.AddAsset(visualizemeshA)

visualizemeshB = fea.ChVisualizationFEAmesh(my_mesh)
visualizemeshB.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
visualizemeshB.SetWireframe(True)
visualizemeshB.SetDrawInUndeformedReference(True)
my_mesh.AddAsset(visualizemeshB)

visualizemeshC = fea.ChVisualizationFEAmesh(my_mesh)
visualizemeshC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
visualizemeshC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
visualizemeshC.SetSymbolsThickness(0.004)
my_mesh.AddAsset(visualizemeshC)

visualizemeshD = fea.ChVisualizationFEAmesh(my_mesh)
visualizemeshD.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_ELEM_TENS_STRAIN)
visualizemeshD.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
visualizemeshD.SetSymbolsScale(1)
visualizemeshD.SetColorscaleMinMax(-0.5, 5)
visualizemeshD.SetZbufferHide(False)
my_mesh.AddAsset(visualizemeshD)

application.AssetBindAll()
application.AssetUpdateAll()

# ----------------------------------
# Perform a dynamic time integration
# ----------------------------------

# Set up solver

solver = chrono.ChSolverMINRES()
my_system.SetSolver(solver)

solver.EnableDiagonalPreconditioner(True)
#solver.SetVerbose(True)

my_system.SetSolverMaxIterations(100)
my_system.SetSolverForceTolerance(1e-10)

# Set up integrator

stepper = chrono.ChTimestepperHHT(my_system)
my_system.SetTimestepper(stepper)

stepper.SetAlpha(-0.2)
stepper.SetMaxiters(5)
stepper.SetAbsTolerances(1e-5)
stepper.SetMode(chrono.ChTimestepperHHT.POSITION)
stepper.SetScaling(True)
stepper.SetStepControl(True)
stepper.SetMinStepSize(1e-4)
#stepper.SetVerbose(True)

# Simulation loop

application.SetTimestep(0.01)

while application.GetDevice().run() :
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

