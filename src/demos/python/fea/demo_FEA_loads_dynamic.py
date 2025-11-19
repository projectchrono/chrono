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
import pychrono.irrlicht as chronoirr
import errno
import os
import copy

# Set output root directory
chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

out_dir = chrono.GetChronoOutputPath() + "FEA_Loads_Dynamic/"

print("Copyright (c) 2017 projectchrono.org ")

# Create (if needed) output directory
try:
    os.mkdir(out_dir)
except OSError as exc:
    if exc.errno != errno.EEXIST:
       print("Error creating output directory " )

# Create the physical system
sys = chrono.ChSystemSMC()

# Create a mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Create some nodes (with default mass 0)
nodeA = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(0, 0, 0)))
nodeB = fea.ChNodeFEAxyzrot(chrono.ChFramed(chrono.ChVector3d(2, 0, 0)))
nodeA.SetMass(0.0)
nodeB.SetMass(0.0)
mesh.AddNode(nodeA)
mesh.AddNode(nodeB)

# Create beam section & material
beam_section = fea.ChBeamSectionEulerAdvanced()
beam_wy = 0.1
beam_wz = 0.2
beam_section.SetAsRectangularSection(beam_wy, beam_wz)
beam_section.SetYoungModulus(0.01e9)
beam_section.SetShearModulus(0.01e9 * 0.3)
beam_section.SetRayleighDamping(0.200)
beam_section.SetDensity(1500)

# Create an Eulero-Bernoulli beam with a single element
elementA = fea.ChElementBeamEuler()
elementA.SetNodes(nodeA, nodeB)
elementA.SetSection(beam_section)
mesh.AddElement(elementA)

# Create the ground body
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# Create a constraint the end of the beam
constrA = chrono.ChLinkMateGeneric()
constrA.Initialize(nodeA, ground, False, nodeA.Frame(), nodeA.Frame())
sys.Add(constrA)
constrA.SetConstrainedCoords(True, True, True,   # x, y, z
                              True, True, True)  # Rx, Ry, Rz

# Create the load container
load_container = chrono.ChLoadContainer()
sys.Add(load_container)

# Create a custom load with stiff force, acting on a single node, but
# this time we inherit directly from ChLoadCustom, i.e. a load that does not require ChLoader features.
# This is mostly used in case one does not need the automatic surface/volume quadrature of ChLoader.
# As a stiff load, this will automatically generate a jacobian (tangent stiffness matrix K)
# that will be used in statics, implicit integrators, etc.

print("   Custom load with stiff force, acting on a single node.")

nodeD = fea.ChNodeFEAxyz(chrono.ChVector3d(2, 10, 3))
mesh.AddNode(nodeD)

class MyLoadCustom(chrono.ChLoadCustom):
    def __init__(self, loadable):
        chrono.ChLoadCustom.__init__(self, loadable)

    # "Virtual" copy constructor (covariant return type).
    def Clone(self):
        newinst = copy.deepcopy(self)
        return  newinst

    # Compute Q=Q(x,v)
    # This is the function that you have to implement. It should return the generalized Q load
    # (i.e.the force in generalized lagrangian coordinates).
    # For ChNodeFEAxyz, Q loads are expected as 3-rows vectors, containing absolute force x,y,z.
    # As this is a stiff force field, dependency from state_x and state_y must be considered.
    def ComputeQ(self,         #
                 state_x,      # state position to evaluate Q
                 state_w):     # state speed to evaluate Q
        if not state_x==None and not state_w==None :
            node_pos = chrono.ChVector3d(state_x.GetItem(0), state_x.GetItem(1), state_x.GetItem(2))
            node_vel = chrono.ChVector3d(state_w.GetItem(0), state_w.GetItem(1), state_w.GetItem(2))
        else:
            node = fea.CastToChNodeFEAxyz( fea.CastToChNodeFEAbase( chrono.CastToChNodeBase(self.loadable) ))
            node_pos = node.GetPos()
            node_vel = node.GetPosDt()

        # Just implement a simple force+spring+damper in xy plane,
        # for spring & damper connected to absolute reference
        Kx = 100
        Ky = 400
        Dx = 0.6
        Dy = 0.9
        x_offset = 2
        y_offset = 5
        x_force = 50
        y_force = 0

        # Store the computed generalized forces in this.load_Q, same x,y,z order as in state_w
        self.load_Q.SetItem(0, x_force - Kx * (node_pos.x - x_offset) - Dx * node_vel.x)
        self.load_Q.SetItem(1, y_force - Ky * (node_pos.y - y_offset) - Dy * node_vel.y)
        self.load_Q.SetItem(2, 0.0)

    # Set this as stiff, to enable the Jacobians
    def IsStiff(self) : 
        return True 

# Instance load object, applying to a node, and add to container
custom_load = MyLoadCustom(nodeD)
load_container.Add(custom_load)

# -----------------------------------------------------------------
# Set visualization of the FEM mesh.

beam_visA = chrono.ChVisualShapeFEA()
beam_visA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_MZ)
beam_visA.SetColormapRange(-400, 200)
beam_visA.SetSmoothFaces(True)
beam_visA.SetWireframe(False)
mesh.AddVisualShapeFEA(beam_visA)

beam_visB = chrono.ChVisualShapeFEA()
beam_visB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
beam_visB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
beam_visB.SetSymbolsThickness(0.006)
beam_visB.SetSymbolsScale(0.01)
beam_visB.SetZbufferHide(False)
mesh.AddVisualShapeFEA(beam_visB)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Loads on beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0.5, 0.0, -3.0), chrono.ChVector3d(0.5, 0.0, 0.0))
vis.AddTypicalLights()

# -----------------------------------------------------------------

# Setup a MINRES solver. For FEA one cannot use the default PSOR type solver.
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(200)
solver.SetTolerance(1e-15)
solver.EnableDiagonalPreconditioner(True)
solver.SetVerbose(False)

sys.GetSolver().AsIterative().SetTolerance(1e-13)

# Set integrator
ts = chrono.ChTimestepperEulerImplicitLinearized(sys)
sys.SetTimestepper(ts)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.001)

