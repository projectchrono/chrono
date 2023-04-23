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
# FEA for 3D beams and constraints
#
# =============================================================================

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as pardiso
import pychrono.irrlicht as chronoirr
import os

# Create a motor between the truss and the crank:
class ChFunction_myf (chrono.ChFunction):
    def __init__(self):
         chrono.ChFunction.__init__(self)
    def Get_y(self,x):
        if (x > 0.4):
            return chrono.CH_C_PI
        else:
            return -chrono.CH_C_PI * (1.0 - m.cos(chrono.CH_C_PI * x / 0.4)) / 2.0

# Output directory
out_dir = chrono.GetChronoOutputPath() + "BEAM_BUCKLING"

print( "Copyright (c) 2017 projectchrono.org \n")

# Create a Chrono::Engine physical system
sys = chrono.ChSystemSMC()

L = 1
H = 0.25
K = 0.05
vA = chrono.ChVectorD(0, 0, 0)
vC = chrono.ChVectorD(L, 0, 0)
vB = chrono.ChVectorD(L, -H, 0)
vG = chrono.ChVectorD(L - K, -H, 0)
vd = chrono.ChVectorD(0, 0, 0.0001)

# Create a truss:
body_truss = chrono.ChBody()
body_truss.SetBodyFixed(True)

sys.AddBody(body_truss)

# Attach a 'box' shape asset for visualization.
boxtruss = chrono.ChBoxShape(0.02, 0.2, 0.1)
body_truss.AddVisualShape(boxtruss, chrono.ChFrameD(chrono.ChVectorD(-0.01, 0, 0), chrono.QUNIT))

# Create body for crank
body_crank = chrono.ChBody()

body_crank.SetPos((vB + vG) * 0.5)
sys.AddBody(body_crank)

# Attach a 'box' shape asset for visualization.
boxcrank = chrono.ChBoxShape(K, 0.02, 0.02)
body_crank.AddVisualShape(boxcrank)

motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunction_myf()
motor.SetAngleFunction(myfun)
sys.Add(motor)

# Create a FEM mesh, that is a container for groups
# of elements and their referenced nodes.
mesh = fea.ChMesh()

# Create the horizontal beam (use an IGA-beam finite element type, for example)

beam_wy = 0.10
beam_wz = 0.01

# Create a section for the IGA beam.
# IGA beams require ChBeamSectionCosserat sections, containing at least
# a ChElasticityCosserat and ChInertiaCosserat models, and optional ChDampingCosserat and ChPlasticityCosserat.
minertia = fea.ChInertiaCosseratSimple()
minertia.SetAsRectangularSection(beam_wy, beam_wz, 2700)  # automatically sets A etc., from width, height, density

melasticity = fea.ChElasticityCosseratSimple()
melasticity.SetYoungModulus(73.0e9)
melasticity.SetGwithPoissonRatio(0.3)
melasticity.SetAsRectangularSection(beam_wy, beam_wz)

msection1 = fea.ChBeamSectionCosserat(minertia, melasticity)

msection1.SetDrawThickness(beam_wy, beam_wz)

builder_iga = fea.ChBuilderBeamIGA()
builder_iga.BuildBeam(mesh,           # the mesh to put the elements in
                      msection1,         # section of the beam
                      32,                # number of sections (spans)
                      vA,                # start point
                      vC,                # end point
                      chrono.VECT_Y,     # suggested Y direction of section
                      3)                 # order (3 = cubic, etc)
builder_iga.GetLastBeamNodes().front().SetFixed(True)
node_tip = builder_iga.GetLastBeamNodes()[-1]
node_mid = builder_iga.GetLastBeamNodes()[17]

# Create the vertical beam (Here use Euler beams, for example).
section2 = fea.ChBeamSectionEulerAdvanced()

hbeam_d = 0.024
section2.SetDensity(2700)
section2.SetYoungModulus(73.0e9)
section2.SetGwithPoissonRatio(0.3)
section2.SetBeamRaleyghDamping(0.000)
section2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(mesh,               # the mesh where to put the created nodes and elements
                   section2,             # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                   3,                     # the number of ChElementBeamEuler to create
                   vC + vd,               # the 'A' poin space (beginning of beam)
                   vB + vd,               # the 'B' poin space (end of beam)
                   chrono.ChVectorD(1, 0, 0))  # the 'Y' up direction of the section for the beam
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constrabetween the vertical and horizontal beams:
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
sys.Add(constr_bb)

constr_bb.SetConstrainedCoords(True, True, True,      # x, y, z
                                False, False, False)  # Rx, Ry, Rz

# For example, attach small shape to show the constraint
sphereconstr2 = chrono.ChSphereShape(0.01)
constr_bb.AddVisualShape(sphereconstr2)

# Create a beam as a crank
section3 = fea.ChBeamSectionEulerAdvanced()

crankbeam_d = 0.048
section3.SetDensity(2700)
section3.SetYoungModulus(73.0e9)
section3.SetGwithPoissonRatio(0.3)
section3.SetBeamRaleyghDamping(0.000)
section3.SetAsCircularSection(crankbeam_d)
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(mesh,               # the mesh where to put the created nodes and elements
                   section3,             # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                   3,                     # the number of ChElementBeamEuler to create
                   vG + vd,               # the 'A' poin space (beginning of beam)
                   vB + vd,               # the 'B' poin space (end of beam)
                   chrono.ChVectorD(0, 1, 0))  # the 'Y' up direction of the section for the beam

node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]
# Create a constraint between the crank beam and body crank:
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
sys.Add(constr_cbd)

constr_cbd.SetConstrainedCoords(True, True, True,   # x, y, z
                                 True, True, True)  # Rx, Ry, Rz

# Create a constrabetween the vertical beam and the crank beam:
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
sys.Add(constr_bc)

constr_bc.SetConstrainedCoords(True, True, True,    # x, y, z
                                True, True, False)  # Rx, Ry, Rz

# For example, attach small shape to show the constraint
sphereconstr3 = chrono.ChSphereShape(0.01)
constr_bc.AddVisualShape(sphereconstr3)

#
# Final touches..
#

# We do not want gravity effect on FEA elements in this demo
mesh.SetAutomaticGravity(False)

# Remember to add the mesh to the system!
sys.Add(mesh)

# ==Asset== attach a visualization of the FEM mesh.
# This will automatically update a triangle mesh (a ChTriangleMeshShape
# asset that is internally managed) by setting  proper
# coordinates and vertex colors as in the FEM elements.
# Such triangle mesh can be rendered by Irrlicht or POVray or whatever
# postprocessor that can handle a colored ChTriangleMeshShape).

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.01)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Beams and constraints')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.0, 0.6, -1.0))
vis.AddTypicalLights()

# Use a solver that can handle stiffnss matrices:
pardiso_solver = pardiso.ChSolverPardisoMKL()
sys.SetSolver(pardiso_solver)

# Use the following for less numerical damping, 2nd order accuracy (but slower)
ts = chrono.ChTimestepperHHT(sys)
ts.SetStepControl(False)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawGrid(vis,
        0.05, 0.05, 20, 20, 
        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngZ(chrono.CH_C_PI_2)))
    vis.EndScene()

    sys.DoStepDynamics(0.001)
