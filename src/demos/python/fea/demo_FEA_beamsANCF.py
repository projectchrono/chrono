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
# Authors: Mike Taylor and Radu Serban
# =============================================================================
#
# Small Displacement, Small Deformation, Linear Isotropic Benchmark test for
# ANCF beam elements - Square cantilevered beam with a time-dependent tip load
#
# Garcia-Vallejo, D., Mayo, J., Escalona, J. L., & Dominguez, J. (2004).
# Efficient evaluation of the elastic forces and the Jacobian in the absolute
# nodal coordinate formulation. Nonlinear Dynamics, 35(4), 313-329.
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr
import copy
import math

def PopulateMesh_beamANCF_3333(mesh, material, dimensions) :
    print("Use ChElementBeamANCF_3333")

    # Extract beam dimensions
    length = dimensions.x
    thickness = dimensions.y
    width = dimensions.z

    # Populate the mesh container with the nodes and elements for the meshed beam
    num_elements = 8
    num_nodes = (2 * num_elements) + 1
    dx = length / (num_nodes - 1)

    # Setup beam cross section gradients to initially align with the global y and z directions
    dir1 = chrono.ChVector3d(0, 1, 0)
    dir2 = chrono.ChVector3d(0, 0, 1)

    # Create the first node and fix it completely to ground (Cantilever constraint)
    nodeA = fea.ChNodeFEAxyzDD(chrono.ChVector3d(0, 0, 0.0), dir1, dir2)
    nodeA.SetFixed(True)
    mesh.AddNode(nodeA)

    last_element = fea.ChElementBeamANCF_3333()

    for i in range(1,num_elements+1):
        nodeB = fea.ChNodeFEAxyzDD(chrono.ChVector3d(dx * (2 * i), 0, 0), dir1, dir2)
        nodeC = fea.ChNodeFEAxyzDD(chrono.ChVector3d(dx * (2 * i - 1), 0, 0), dir1, dir2)
        mesh.AddNode(nodeB)
        mesh.AddNode(nodeC)

        element = fea.ChElementBeamANCF_3333()
        element.SetNodes(nodeA, nodeB, nodeC)
        element.SetDimensions(2 * dx, thickness, width)
        element.SetMaterial(material)
        element.SetAlphaDamp(0.0)
        mesh.AddElement(element)

        nodeA = nodeB
        last_element = element

    return last_element

def PopulateMesh_beamANCF_3243(mesh, material, dimensions) :
    print("Use ChElementBeamANCF_3243")

    # Extract beam dimensions
    length = dimensions.x
    thickness = dimensions.y
    width = dimensions.z

    # Populate the mesh container with the nodes and elements for the meshed beam
    num_elements = 8
    num_nodes = num_elements + 1
    dx = length / (num_nodes - 1)

    # Create the first node and fix it completely to ground (Cantilever constraint)
    nodeA = fea.ChNodeFEAxyzDDD(chrono.ChVector3d(0, 0, 0.0))
    nodeA.SetFixed(True)
    mesh.AddNode(nodeA)

    last_element = fea.ChElementBeamANCF_3243()

    for i in range(1,num_elements+1):
        nodeB = fea.ChNodeFEAxyzDDD(chrono.ChVector3d(dx * i, 0, 0))
        mesh.AddNode(nodeB)

        element = fea.ChElementBeamANCF_3243()
        element.SetNodes(nodeA, nodeB)
        element.SetDimensions(dx, thickness, width)
        element.SetMaterial(material)
        element.SetAlphaDamp(0.0)
        mesh.AddElement(element)

        nodeA = nodeB
        last_element = element

    return last_element

# =====================================================

# Select element type
print("Options:")
print("1  : ElementBeamANCF_3243")
print("2  : ElementBeamANCF_3333")
element_type = int(input("Select option: "))

# Create containing system
sys = chrono.ChSystemSMC()
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.8))

# Set up solver
solver = mkl.ChSolverPardisoMKL()
solver.UseSparsityPatternLearner(True)
solver.LockSparsityPattern(True)
solver.SetVerbose(False)
sys.SetSolver(solver)

# Set up integrator
integrator = chrono.ChTimestepperHHT(sys)
integrator.SetAlpha(-0.2)
integrator.SetMaxIters(100)
integrator.SetAbsTolerances(1e-5)
integrator.SetVerbose(False)
integrator.SetModifiedNewton(True)
sys.SetTimestepper(integrator)

# Mesh properties
length = 5       # m
width = 0.1      # m
thickness = 0.1  # m
rho = 8000       # kg/m^3
E = 4e8          # Pa
nu = 0           # Poisson effect neglected for this model
# Timoshenko shear correction coefficients for a rectangular cross-section
k1 = 10 * (1 + nu) / (12 + 11 * nu)
k2 = k1

material = fea.ChMaterialBeamANCF(rho, E, nu, k1, k2)

# Create mesh container
mesh = fea.ChMesh()
mesh.SetAutomaticGravity(False)
sys.Add(mesh);

# Create the load container and add to the current system
load_container = chrono.ChLoadContainer()
sys.Add(load_container)

# Define a custom point load with a time-dependent force
class MyLoaderTimeDependentTipLoad(chrono.ChLoaderUatomic):
    def __init__(self, loadable):
        chrono.ChLoaderUatomic.__init__(self, loadable)

    # "Virtual" copy constructor (covariant return type)
    def Clone(self):
        newinst = copy.deepcopy(self)
        return  newinst

    def ComputeF(self,         #
                 U,            # normalized position along the beam axis [-1...1]
                 F,            # load at U (set to zero on entry)
                 state_x,      # state position to evaluate Q
                 state_w):     # state speed to evaluate Q
        t = self.m_sys.GetChTime()

        Fmax = -500
        tc = 2
        Fz = Fmax
        if (t < tc):
            Fz = 0.5 * Fmax * (1 - math.cos(math.pi * t / tc))

        F.SetItem(2, Fz)  # Apply the force along the global Z axis

    def SetSystem(self, sys):
        self.m_sys = sys

# Populate mesh with beam elements of specified type.
# Create a custom load that uses the custom loader above
element_name = '';
if element_type == 1:
    element_name = 'ANCF_3243'

    last_element = PopulateMesh_beamANCF_3243(mesh, material, chrono.ChVector3d(length, thickness, width))

    loader = MyLoaderTimeDependentTipLoad(last_element)
    loader.SetSystem(sys)        # set containing system
    loader.SetApplication(1.0)   # specify application point
    load = chrono.ChLoad(loader)
    load_container.Add(load)     # add the load to the load container.
elif element_type == 2:
    element_name = 'ANCF_3333'
    
    last_element = PopulateMesh_beamANCF_3333(mesh, material, chrono.ChVector3d(length, thickness, width))
    
    loader = MyLoaderTimeDependentTipLoad(last_element)
    loader.SetSystem(sys)        # set containing system
    loader.SetApplication(1.0)   # specify application point
    load = chrono.ChLoad(loader)
    load_container.Add(load)     # add the load to the load container.

# Set visualization of the FEM mesh.
beam_visA = chrono.ChVisualShapeFEA(mesh)
beam_visA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
beam_visA.SetWireframe(True)
beam_visA.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(beam_visA)

beam_visB = chrono.ChVisualShapeFEA(mesh)
beam_visB.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
beam_visB.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
beam_visB.SetSymbolsThickness(0.01)
mesh.AddVisualShapeFEA(beam_visB)

# Create the Irrlicht visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(800,600)
vis.SetWindowTitle('ANCF beam ' + element_name)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(-0.4, 0.4, 0.4), chrono.ChVector3d(0.0, 0.0, 0.0))
vis.AddTypicalLights()

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.01)
