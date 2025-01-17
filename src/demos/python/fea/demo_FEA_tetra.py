# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2024 projectchrono.org
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

print( "Copyright (c) 2024 projectchrono.org")
print( "-----------------------------------------------------------")
print( "TEST: tetrahedron FEM dynamics, implicit integration")
print( "-----------------------------------------------------------")

sys = chrono.ChSystemSMC()

# FEA mesh
mesh = fea.ChMesh()
sys.Add(mesh)

# Continuum material
material = fea.ChContinuumElastic()
material.SetYoungModulus(1e7)
material.SetPoissonRatio(0.3)
material.SetRayleighDampingBeta(0.01)
material.SetDensity(1000)

# FEA nodes
node1 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 0))
node2 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 0, 1))
node3 = fea.ChNodeFEAxyz(chrono.ChVector3d(0, 1, 0))
node4 = fea.ChNodeFEAxyz(chrono.ChVector3d(1, 0, 0))

node1.SetMass(200)
node2.SetMass(200)
node3.SetMass(200)
node4.SetMass(200)

node3.SetPos(node3.GetX0() + chrono.ChVector3d(0, 0.01, 0));

mesh.AddNode(node1)
mesh.AddNode(node2)
mesh.AddNode(node3)
mesh.AddNode(node4)

# FEA elements
element1 = fea.ChElementTetraCorot_4()
element1.SetNodes(node1, node2, node3, node4)
element1.SetMaterial(material)

mesh.AddElement(element1)

# Ground body
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# Constraints (nodes - ground)
constraint1 = fea.ChLinkNodeFrame();
constraint2 = fea.ChLinkNodeFrame();
constraint3 = fea.ChLinkNodeFrame();

constraint1.Initialize(node1, ground)
constraint2.Initialize(node2, ground)
constraint3.Initialize(node4, ground)

sys.Add(constraint1)
sys.Add(constraint2)
sys.Add(constraint3)

# FEA mesh visualization settings
fea_vis_wireframe = chrono.ChVisualShapeFEA()
fea_vis_wireframe.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
fea_vis_wireframe.SetWireframe(True)
fea_vis_wireframe.SetDrawInUndeformedReference(True)
mesh.AddVisualShapeFEA(fea_vis_wireframe)

fea_vis_surface = chrono.ChVisualShapeFEA()
fea_vis_surface.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NONE)
fea_vis_surface.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_SURFACE)
fea_vis_surface.SetZbufferHide(False)
mesh.AddVisualShapeFEA(fea_vis_surface)

# Create run-time visualization system
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Tetra Element')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, 0.25, -0.25), chrono.ChVector3d(0, 0.25, 0))
vis.AddTypicalLights()

# Set solver and integrator
solver = chrono.ChSolverMINRES()
sys.SetSolver(solver)
solver.SetMaxIterations(100)
solver.SetTolerance(1e-12)

sys.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)

# Simulation loop
while vis.Run():
    vis.BeginScene()
    vis.Render()
    chronoirr.drawCoordsys(vis, chrono.CSYSNORM, 1.2)
    vis.EndScene()
    sys.DoStepDynamics(0.001)

    time = sys.GetChTime()
    if time < 0.1:
        print ('t = ', time, ' node3 y_pos =', node3.GetPos().y )
