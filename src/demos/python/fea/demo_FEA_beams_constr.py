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
# FEA for 3D beams and constraints
#
# =============================================================================

import math as m
import pychrono as chrono
import pychrono.fea as fea
import pychrono.mkl as mkl
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
my_system = chrono.ChSystemSMC()

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(my_system, "Beams and constraints", chronoirr.dimension2du(800, 600), False, True)

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(0.0, 0.6, -1.0))

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

my_system.AddBody(body_truss)

# Attach a 'box' shape asset for visualization.
mboxtruss = chrono.ChBoxShape()
mboxtruss.GetBoxGeometry().Pos = chrono.ChVectorD(-0.01, 0, 0)
mboxtruss.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.02, 0.2, 0.1))
body_truss.AddAsset(mboxtruss)

# Create body for crank
body_crank = chrono.ChBody()

body_crank.SetPos((vB + vG) * 0.5)
my_system.AddBody(body_crank)

# Attach a 'box' shape asset for visualization.
mboxcrank = chrono.ChBoxShape()
mboxcrank.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, 0)
mboxcrank.GetBoxGeometry().SetLengths(chrono.ChVectorD(K, 0.02, 0.02))
body_crank.AddAsset(mboxcrank)

motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(body_truss, body_crank, chrono.ChFrameD(vG))
myfun = ChFunction_myf()
motor.SetAngleFunction(myfun)
my_system.Add(motor)

# Create a FEM mesh, that is a container for groups
# of elements and their referenced nodes.
my_mesh = fea.ChMesh()

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
builder_iga.BuildBeam(my_mesh,           # the mesh to put the elements in
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
msection2 = fea.ChBeamSectionEulerAdvanced()

hbeam_d = 0.024
msection2.SetDensity(2700)
msection2.SetYoungModulus(73.0e9)
msection2.SetGwithPoissonRatio(0.3)
msection2.SetBeamRaleyghDamping(0.000)
msection2.SetAsCircularSection(hbeam_d)

builderA = fea.ChBuilderBeamEuler()
builderA.BuildBeam(my_mesh,               # the mesh where to put the created nodes and elements
                  msection2,             # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                  3,                     # the number of ChElementBeamEuler to create
                  vC + vd,               # the 'A' poin space (beginning of beam)
                  vB + vd,               # the 'B' poin space (end of beam)
                  chrono.ChVectorD(1, 0, 0))  # the 'Y' up direction of the section for the beam
node_top = builderA.GetLastBeamNodes()[0]
node_down = builderA.GetLastBeamNodes()[-1]

# Create a constrabetween the vertical and horizontal beams:
constr_bb = chrono.ChLinkMateGeneric()
constr_bb.Initialize(node_top, node_tip, False, node_top.Frame(), node_top.Frame())
my_system.Add(constr_bb)

constr_bb.SetConstrainedCoords(True, True, True,      # x, y, z
                                False, False, False)  # Rx, Ry, Rz

# For example, attach small shape to show the constraint
msphereconstr2 = chrono.ChSphereShape()
msphereconstr2.GetSphereGeometry().rad = 0.01
constr_bb.AddAsset(msphereconstr2)

# Create a beam as a crank
msection3 = fea.ChBeamSectionEulerAdvanced()

crankbeam_d = 0.048
msection3.SetDensity(2700)
msection3.SetYoungModulus(73.0e9)
msection3.SetGwithPoissonRatio(0.3)
msection3.SetBeamRaleyghDamping(0.000)
msection3.SetAsCircularSection(crankbeam_d)
builderB = fea.ChBuilderBeamEuler()
builderB.BuildBeam(my_mesh,               # the mesh where to put the created nodes and elements
                  msection3,             # the ChBeamSectionEulerAdvanced to use for the ChElementBeamEuler elements
                  3,                     # the number of ChElementBeamEuler to create
                  vG + vd,               # the 'A' poin space (beginning of beam)
                  vB + vd,               # the 'B' poin space (end of beam)
                  chrono.ChVectorD(0, 1, 0))  # the 'Y' up direction of the section for the beam

node_crankG = builderB.GetLastBeamNodes()[0]
node_crankB = builderB.GetLastBeamNodes()[-1]
# Create a constraint between the crank beam and body crank:
constr_cbd = chrono.ChLinkMateGeneric()
constr_cbd.Initialize(node_crankG, body_crank, False, node_crankG.Frame(), node_crankG.Frame())
my_system.Add(constr_cbd)

constr_cbd.SetConstrainedCoords(True, True, True,   # x, y, z
                                 True, True, True)  # Rx, Ry, Rz

# Create a constrabetween the vertical beam and the crank beam:
constr_bc = chrono.ChLinkMateGeneric()
constr_bc.Initialize(node_down, node_crankB, False, node_crankB.Frame(), node_crankB.Frame())
my_system.Add(constr_bc)

constr_bc.SetConstrainedCoords(True, True, True,    # x, y, z
                                True, True, False)  # Rx, Ry, Rz

# For example, attach small shape to show the constraint
msphereconstr3 = chrono.ChSphereShape()
msphereconstr3.GetSphereGeometry().rad = 0.01
constr_bc.AddAsset(msphereconstr3)

#
# Final touches..
#

# We do not want gravity effect on FEA elements in this demo
my_mesh.SetAutomaticGravity(False)

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
mvisualizebeamA.SetFEMdataType(fea.ChVisualizationFEAmesh.E_PLOT_ELEM_BEAM_MX)
mvisualizebeamA.SetColorscaleMinMax(-500, 500)
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

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.

application.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

application.AssetUpdateAll()

# SIMULATION LOOP

# Use a solver that can handle stiffnss matrices:
mkl_solver = mkl.ChSolverMKL()
my_system.SetSolver(mkl_solver)

application.SetTimestep(0.001)
application.SetVideoframeSaveInterval(10)

# Use the following for less numerical damping, 2nd order accuracy (but slower)
ts = chrono.ChTimestepperHHT(my_system)
ts.SetStepControl(False)
my_system.SetTimestepper(ts)
# Output data
if not os.path.isdir(out_dir):
    print("Error creating directory " )


filename = out_dir + "/buckling_mid.dat"
#file_out1 = chrono.ChStreamOutAsciiFile(filename)

while (application.GetDevice().run()):
    application.BeginScene()

    application.DrawAll()

    chronoirr.ChIrrTools.drawGrid(application.GetVideoDriver(), 0.05, 0.05, 20, 20, chrono.ChCoordsysD(chrono.VNULL, chrono.CH_C_PI_2, chrono.VECT_Z),
                         chronoirr.SColor(50, 90, 90, 90), True)

    application.DoStep()

    # Save output for the first 0.4 seconds
    #if (application.GetSystem().GetChTime() <= 0.4): 
        #file_out1(application.GetSystem().GetChTime() + " " + node_mid.GetPos().z() + " " + node_mid.GetWvel_par().x() + "\n")
    application.EndScene()
