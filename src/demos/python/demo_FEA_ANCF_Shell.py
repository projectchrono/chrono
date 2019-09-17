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
print("-----------------------------------------------------------\n")
print("     ANCF Shell Elements demo with implicit integration \n")
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
numDiv_y = 10
numDiv_z = 10
N_x = numDiv_x + 1
N_y = numDiv_y + 1
N_z = numDiv_z + 1
# Number of elements in the z direction is considered as 1
TotalNumElements = numDiv_x * numDiv_y
TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1)
# For uniform mesh
dx = plate_lenght_x / numDiv_x
dy = plate_lenght_y / numDiv_y
dz = plate_lenght_z / numDiv_z

# Create and add the nodes
for i in range(TotalNumNodes) :
    # Node location
    # x incremented by dx at each iteration reastart at each 11*n
    loc_x = (i % (numDiv_x + 1)) * dx
    # y incremented by dy every Div_x iterations.
    loc_y = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy
    loc_z = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz

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
    node0 = int( i % numDiv_x + (numDiv_x+1) * math.floor(i/numDiv_x))
    node1 = int(i % numDiv_x + 1 + (numDiv_x+1) * math.floor(i/numDiv_x))
    node2 = int(i % numDiv_x + 1 + N_x + (numDiv_x+1) * math.floor(i/numDiv_x))
    node3 = int(i % numDiv_x + N_x + (numDiv_x+1) * math.floor(i/numDiv_x))

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

mvisualizemesh = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_SPEED_NORM)
mvisualizemesh.SetColorscaleMinMax(0.0, 5.50)
mvisualizemesh.SetShrinkElements(True, 0.85)
mvisualizemesh.SetSmoothFaces(True)
my_mesh.AddAsset(mvisualizemesh)

mvisualizemeshref = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshref.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizemeshref.SetWireframe(True)
mvisualizemeshref.SetDrawInUndeformedReference(True)
my_mesh.AddAsset(mvisualizemeshref)

mvisualizemeshC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizemeshC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizemeshC.SetSymbolsThickness(0.004)
my_mesh.AddAsset(mvisualizemeshC)

mvisualizemeshD = fea.ChVisualizationFEAmesh(my_mesh)
# mvisualizemeshD.SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_VECT_SPEED)
mvisualizemeshD.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_ELEM_TENS_STRAIN)
mvisualizemeshD.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizemeshD.SetSymbolsScale(1)
mvisualizemeshD.SetColorscaleMinMax(-0.5, 5)
mvisualizemeshD.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizemeshD)

application.AssetBindAll()
application.AssetUpdateAll()

# ----------------------------------
# Perform a dynamic time integration
# ----------------------------------

# Mark completion of system construction
my_system.SetupInitial()

# Set up solver
#my_system.SetSolverType(chrono.ChSolver.Type_MINRES)
#msolver = std::static_pointer_cast<ChSolverMINRES>(my_system.GetSolver())
msolver = chrono.ChSolverMINRES()
my_system.SetSolver(msolver)
msolver.SetDiagonalPreconditioning(True)
my_system.SetMaxItersSolverSpeed(100)
my_system.SetTolForce(1e-10)

# Set up integrator
#my_system.SetTimestepperType(ChTimestepper::Type::HHT)
#auto mystepper = std::static_pointer_cast<ChTimestepperHHT>(my_system.GetTimestepper())
mystepper = chrono.ChTimestepperHHT(my_system)
mystepper.SetAlpha(-0.2)
mystepper.SetMaxiters(100)
mystepper.SetAbsTolerances(1e-5)
mystepper.SetMode(chrono.ChTimestepperHHT.POSITION)
mystepper.SetScaling(True)
application.SetTimestep(0.001)

while application.GetDevice().run() :
    application.BeginScene()
    application.DrawAll()
    application.DoStep()
    application.EndScene()

