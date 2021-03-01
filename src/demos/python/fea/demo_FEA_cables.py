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
# FEA for 3D beams of 'cable' type (ANCF gradient-deficient beams)
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
#import pychrono.mkl as mkl
import pychrono.irrlicht as chronoirr
from cables import Model1, Model2, Model3


# Select solver type (SPARSE_QR, SPARSE_LU, or MINRES).
#ChSolver::Type solver_type = ChSolver::Type::SPARSE_QR
solver = chrono.ChSolverSparseQR()

print("Copyright (c) 2017 projectchrono.org\nChrono version: ")

# Create a Chrono::Engine physical system
my_system = chrono.ChSystemSMC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(my_system, "Cables FEM", chronoirr.dimension2du(800, 600))

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0., 0.6, -1))

# Create a mesh, that is a container for groups of elements and
# their referenced nodes.
my_mesh = fea.ChMesh()

# Create one of the available models (defined in FEAcables.h)
##model = Model1(my_system, my_mesh)
##model = Model2(my_system, my_mesh)
model = Model3(my_system, my_mesh)

# Remember to add the mesh to the system!
my_system.Add(my_mesh)

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChTriangleMeshShape).
# Do not forget AddAsset() at the end!

mvisualizebeamA = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MZ)
mvisualizebeamA.SetColorscaleMinMax(-0.4, 0.4)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
my_mesh.AddAsset(mvisualizebeamA)

mvisualizebeamC = fea.ChVisualizationFEAmesh(my_mesh)
mvisualizebeamC.SetFEMglyphType(fea.ChVisualizationFEAmesh.E_GLYPH_NODE_DOT_POS) # E_GLYPH_NODE_CSYS
mvisualizebeamC.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
my_mesh.AddAsset(mvisualizebeamC)

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.
application.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
application.AssetUpdateAll()

# Set solver and solver settings

if solver.GetType()==chrono.ChSolver.Type_SPARSE_QR:
	print("Using SparseQR solver")
	my_system.SetSolver(solver)
	solver.UseSparsityPatternLearner(True)
	solver.LockSparsityPattern(True)
	solver.SetVerbose(False)

elif solver.GetType()== chrono.ChSolver.Type_MINRES :
	print( "Using MINRES solver" )
	my_system.SetSolver(solver)
	solver.SetMaxIterations(200)
	solver.SetTolerance(1e-10)
	solver.EnableDiagonalPreconditioner(True)
	solver.EnableWarmStart(True)  # IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
	solver.SetVerbose(False)

else:
	print("Solver type not supported." )
    
my_system.SetSolverForceTolerance(1e-13)

# Set integrator
ts = chrono.ChTimestepperEulerImplicitLinearized(my_system)
my_system.SetTimestepper(ts)

# SIMULATION LOOP
application.SetTimestep(0.01)

while (application.GetDevice().run()) :
	application.BeginScene()
	application.DrawAll()
	application.DoStep()
	application.EndScene()
	##model.PrintBodyPositions()

