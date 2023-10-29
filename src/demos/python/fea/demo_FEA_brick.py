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
import numpy as np


print( "Copyright (c) 2017 projectchrono.org")

sys = chrono.ChSystemSMC()


print( "-----------------------------------------------------------")
print( "-----------------------------------------------------------")
print( "     Brick Elements demo with implicit integration ")
print( "-----------------------------------------------------------")

# The physical system: it contains all physical objects.
# Create a mesh, that is a container for groups
# of elements and their referenced nodes.
mesh = fea.ChMesh()
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
NumNodes = np.empty([TotalNumElements, 8], dtype=int)
LayNum = np.empty([TotalNumElements, 1], dtype=int)
NDR = np.empty([TotalNumNodes, 3], dtype=int)
ElemLengthXY = chrono.ChMatrixDynamicD(TotalNumElements, 3)
MPROP = np.empty([10, 12], dtype=float)

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
	mesh.AddNode(node)
	if (NDR[i, 0] == 1 and NDR[i, 1] == 1 and NDR[i, 2] == 1) :
		node.SetFixed(True)
	
	i += 1 

nodetip = fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(TotalNumNodes - 1)))

elemcount = 0
while elemcount < TotalNumElements : 
    element = fea.ChElementHexaANCF_3813()
    InertFlexVec  = chrono.ChVectorD(ElemLengthXY[elemcount, 0], ElemLengthXY[elemcount, 1], ElemLengthXY[elemcount, 2]) # read element length
    element.SetInertFlexVec(InertFlexVec)
    element.SetNodes(fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 0])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 1])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 2])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 3])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 4])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 5])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 6])))),
					  fea.CastToChNodeFEAxyz(fea.CastToChNodeFEAbase(mesh.GetNode(int(NumNodes[elemcount, 7])))))

    element.SetMaterial(mmaterial)
    element.SetElemNum(elemcount)            # for EAS
    element.SetMooneyRivlin(False)           # turn on/off Mooney Rivlin (Linear Isotropic by default)
    stock_alpha_EAS = np.zeros(9) 
    element.SetStockAlpha(stock_alpha_EAS[0], stock_alpha_EAS[1], stock_alpha_EAS[2],
                               stock_alpha_EAS[3], stock_alpha_EAS[4], stock_alpha_EAS[5],
                               stock_alpha_EAS[6], stock_alpha_EAS[7], stock_alpha_EAS[8])
    mesh.AddElement(element)
    elemcount += 1

sys.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))
# Remember to add the mesh to the system!
sys.Add(mesh)

# Options for visualization in irrlicht
mvisualizemesh = chrono.ChVisualShapeFEA(mesh)
mvisualizemesh.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NODE_P)
mvisualizemesh.SetShrinkElements(True, 0.85)
mvisualizemesh.SetSmoothFaces(False)
mesh.AddVisualShapeFEA(mvisualizemesh)

mvisualizemeshref = chrono.ChVisualShapeFEA(mesh)
mvisualizemeshref.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizemeshref.SetWireframe(True)
mvisualizemeshref.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(mvisualizemeshref)

mvisualizemeshC = chrono.ChVisualShapeFEA(mesh)
mvisualizemeshC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizemeshC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizemeshC.SetSymbolsThickness(0.015)
mesh.AddVisualShapeFEA(mvisualizemeshC)

mvisualizemeshD = chrono.ChVisualShapeFEA(mesh)
mvisualizemeshD.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NONE)
mvisualizemeshD.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
mvisualizemeshD.SetSymbolsScale(1)
mvisualizemeshD.SetColorscaleMinMax(-0.5, 5)
mvisualizemeshD.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizemeshD)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Brick Elements')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(1.2, 0.6, 0.3), chrono.ChVectorD(0.2, -0.2, 0))
vis.AddTypicalLights()


# Perform a dynamic time integration:

solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(1000)
solver.SetTolerance(1e-10)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(False)

mystepper = chrono.ChTimestepperHHT(sys)
sys.SetTimestepper(mystepper)
mystepper.SetAlpha(-0.2)
mystepper.SetMaxiters(100)
mystepper.SetAbsTolerances(1e-2)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.004)


