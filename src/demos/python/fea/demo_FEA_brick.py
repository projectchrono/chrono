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
# FEA using the brick element
#
# =============================================================================

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.mkl as mkl
import pychrono.irrlicht as chronoirr
import numpy as np


print( "Copyright (c) 2017 projectchrono.org")

my_system = chrono.ChSystemSMC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(my_system, "Brick Elements", chronoirr.dimension2du(800, 600))

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()

application.AddTypicalCamera(chronoirr.vector3df(1.2, 0.6, 0.3),   # camera location
							 chronoirr.vector3df(0.2, -0.2, 0.))   # "look at" location

print( "-----------------------------------------------------------")
print( "-----------------------------------------------------------")
print( "     Brick Elements demo with implicit integration ")
print( "-----------------------------------------------------------")

# The physical system: it contains all physical objects.
# Create a mesh, that is a container for groups
# of elements and their referenced nodes.
my_mesh = fea.ChMesh()
# Geometry of the plate
plate_lenght_x = 1
plate_lenght_y = 1
plate_lenght_z = 0.05  # small thickness
# Specification of the mesh
numDiv_x = 4
numDiv_y = 4
numDiv_z = 1
N_x = numDiv_x + 1
N_y = numDiv_y + 1
N_z = numDiv_z + 1
# Number of elements in the z direction is considered as 1
TotalNumElements = numDiv_x * numDiv_y
TotalNumNodes = (numDiv_x + 1) * (numDiv_y + 1) * (numDiv_z + 1)
# For uniform mesh
dx = plate_lenght_x / numDiv_x
dy = plate_lenght_y / numDiv_y
dz = plate_lenght_z / numDiv_z
MaxMNUM = 1
MTYPE = 1
MaxLayNum = 1

COORDFlex = chrono.ChMatrixDynamicD(TotalNumNodes, 3)
VELCYFlex = chrono.ChMatrixDynamicD(TotalNumNodes, 3)
NumNodes = np.empty([TotalNumElements, 8], dtype=np.int)
LayNum = np.empty([TotalNumElements, 1], dtype=np.uint)
NDR = np.empty([TotalNumNodes, 3], dtype=np.int)
ElemLengthXY = chrono.ChMatrixDynamicD(TotalNumElements, 3)
MPROP = np.empty([10, 12], dtype=np.double)

#!------------------------------------------------!
#!------------ Read Material Data-----------------!
#!------------------------------------------------!

for i in range(MaxMNUM):
	MPROP[i, 0] = 500      # Density [kg/m3]
	MPROP[i, 1] = 2.1E+05  # H(m)
	MPROP[i, 2] = 0.3      # nu

mmaterial = fea.ChContinuumElastic()
mmaterial.Set_RayleighDampingK(0.0)
mmaterial.Set_RayleighDampingM(0.0)
mmaterial.Set_density(MPROP[0, 0])
mmaterial.Set_E(MPROP[0, 1])
mmaterial.Set_G(MPROP[0, 1] / (2 + 2 * MPROP[0, 2]))
mmaterial.Set_v(MPROP[0, 2])
#!------------------------------------------------!
#!--------------- Element data--------------------!
#!------------------------------------------------!
for i in range(TotalNumElements):
	# All the elements belong to the same layer, e.g layer number 1.
	LayNum[i, 0] = 1
	# Node number of the 4 nodes which creates element i.
	# The nodes are distributed this way. First in the x direction for constant
	# y when max x is reached go to the
	# next level for y by doing the same   distribution but for y+1 and keep
	# doing until y limit is reached. Node
	# number start from 1.

	NumNodes[i, 0] = m.floor(i / (numDiv_x)) * (N_x) + i % numDiv_x
	NumNodes[i, 1] = m.floor(i / (numDiv_x)) * (N_x) + i % numDiv_x + 1
	NumNodes[i, 2] = m.floor(i / (numDiv_x)) * (N_x) + i % numDiv_x + 1 + N_x
	NumNodes[i, 3] = m.floor(i / (numDiv_x)) * (N_x) + i % numDiv_x + N_x
	NumNodes[i, 4] = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes[i, 0]
	NumNodes[i, 5] = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes[i, 1]
	NumNodes[i, 6] = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes[i, 2]
	NumNodes[i, 7] = (numDiv_x + 1) * (numDiv_y + 1) + NumNodes[i, 3]

	# Let's keep the element length a fixed number in both direction. (uniform
	# distribution of nodes in both direction)

	ElemLengthXY[i, 0] = dx
	ElemLengthXY[i, 1] = dy
	ElemLengthXY[i, 2] = dz

	if MaxLayNum < LayNum[i, 0] :
		MaxLayNum = LayNum[i, 0]

#!----------------------------------------------!
#!--------- NDR,COORDFlex,VELCYFlex-------------!
#!----------------------------------------------!

for i in range(TotalNumNodes) :
	# If the node is the first node from the left side fix the x,y,z degree of
	# freedom. 1 for constrained 0 for ...
	#-The NDR array is used to define the degree of freedoms that are
	# constrained in the 6 variable explained above.
    NDR[i, 0] = 1 if (i % (numDiv_x + 1) == 0) else 0
    NDR[i, 1] = 1 if (i % (numDiv_x + 1) == 0) else 0
    NDR[i, 2] = 1 if (i % (numDiv_x + 1) == 0) else 0
    #-COORDFlex are the initial coordinates for each node,
    # the first three are the position
    COORDFlex[i, 0] = (i % (numDiv_x + 1)) * dx
    COORDFlex[i, 1] = (i / (numDiv_x + 1)) % (numDiv_y + 1) * dy
    COORDFlex[i, 2] = (i) / ((numDiv_x + 1) * (numDiv_y + 1)) * dz
    #-VELCYFlex is essentially the same as COORDFlex, but for the initial
    # velocity instead of position.
    # let's assume zero initial velocity for nodes
    VELCYFlex[i, 0] = 0
    VELCYFlex[i, 1] = 0
    VELCYFlex[i, 2] = 0


# Adding the nodes to the mesh
i = 0
while i < TotalNumNodes :
	node = fea.ChNodeFEAxyz(chrono.ChVectorD(COORDFlex[i, 0], COORDFlex[i, 1], COORDFlex[i, 2]))
	node.SetMass(0.0)
	my_mesh.AddNode(node)
	if (NDR[i, 0] == 1 and NDR[i, 1] == 1 and NDR[i, 2] == 1) :
		node.SetFixed(True)
	
	i += 1 

nodetip = fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(TotalNumNodes - 1)))

elemcount = 0
while elemcount < TotalNumElements : 
    element = fea.ChElementBrick()
    InertFlexVec  = chrono.ChVectorD(ElemLengthXY[elemcount, 0], ElemLengthXY[elemcount, 1], ElemLengthXY[elemcount, 2]) # read element length, used in ChElementBrick
    element.SetInertFlexVec(InertFlexVec)
    element.SetNodes(fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 0])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 1])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 2])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 3])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 4])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 5])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 6])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(my_mesh.GetNode(int(NumNodes[elemcount, 7])))))

    element.SetMaterial(mmaterial)
    element.SetElemNum(elemcount)            # for EAS
    element.SetGravityOn(True)               # turn gravity on/off from within the element
    element.SetMooneyRivlin(False)           # turn on/off Mooney Rivlin (Linear Isotropic by default)
    stock_alpha_EAS = np.zeros(9) 
    element.SetStockAlpha(stock_alpha_EAS[0], stock_alpha_EAS[1], stock_alpha_EAS[2],
                               stock_alpha_EAS[3], stock_alpha_EAS[4], stock_alpha_EAS[5],
                               stock_alpha_EAS[6], stock_alpha_EAS[7], stock_alpha_EAS[8])
    my_mesh.AddElement(element)
    elemcount += 1

# Deactivate automatic gravity in mesh
my_mesh.SetAutomaticGravity(False)
my_system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
# Remember to add the mesh to the system!
my_system.Add(my_mesh)

# Options for visualization in irrlicht
mvisualizemesh = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemesh.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NODE_P)
mvisualizemesh.SetShrinkElements(True, 0.85)
mvisualizemesh.SetSmoothFaces(False)
my_mesh.AddAsset(mvisualizemesh)

mvisualizemeshref = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshref.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizemeshref.SetWireframe(True)
mvisualizemeshref.SetDrawInUndeformedReference(True)
my_mesh.AddAsset(mvisualizemeshref)

mvisualizemeshC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS)
mvisualizemeshC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizemeshC.SetSymbolsThickness(0.015)
my_mesh.AddAsset(mvisualizemeshC)

mvisualizemeshD = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizemeshD.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NONE)
mvisualizemeshD.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_SURFACE)
mvisualizemeshD.SetSymbolsScale(1)
mvisualizemeshD.SetColorscaleMinMax(-0.5, 5)
mvisualizemeshD.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizemeshD)

application.AssetBindAll()
application.AssetUpdateAll()

# Perform a dynamic time integration:

solver = chrono.ChSolverMINRES()
my_system.SetSolver(solver)
solver.SetMaxIterations(1000)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(False)

mystepper = chrono.ChTimestepperHHT(my_system)
my_system.SetTimestepper(mystepper)
mystepper.SetAlpha(-0.2)
mystepper.SetMaxiters(100)
mystepper.SetAbsTolerances(1e-5)
mystepper.SetMode(chrono.ChTimestepperHHT.POSITION)
mystepper.SetScaling(True)
application.SetTimestep(0.004)

while application.GetDevice().run() :
	application.BeginScene()
	application.DrawAll()
	application.DoStep()
	application.EndScene()


