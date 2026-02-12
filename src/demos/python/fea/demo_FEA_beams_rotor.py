# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# FEA nonlinear static static_analysis of 3D beams, including centrifugal effect
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
import pychrono.pardisomkl as mkl
import pychrono.irrlicht as chronoirr

print("Timoshenko beam example");

beam_Young = 100.e6
beam_density = 400
beam_wz = 0.3
beam_wy = 0.02
beam_Rmax = 6.2
beam_Rmin = 0.2
rad_s = 3
tip_abs_force = chrono.ChVector3d(0, 0, -36.4)

# Chrono physical system
sys = chrono.ChSystemNSC()

# Base and tower body
tower = chrono.ChBodyEasyBox(10, 2, 10, 3000)
tower.SetFixed(True)
tower.SetPos(chrono.ChVector3d(0, -10, 0))
sys.Add(tower)

# Attach a cylinder shape asset for visualization of the tower
tower_shape = chrono.ChVisualShapeCylinder(0.2, 9.0)
tower.AddVisualShape(tower_shape, chrono.ChFramed(chrono.ChVector3d(0, 5.5, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Rotating hub
hub = chrono.ChBody()
hub.SetPos(chrono.ChVector3d(0, 0, 1))
hub.SetRot(chrono.QuatFromAngleX(chrono.CH_PI_2))
sys.Add(hub)

# Hub motor
motor = chrono.ChLinkMotorRotationAngle()
motor.Initialize(hub, tower, chrono.ChFramed(chrono.ChVector3d(0, 0, 1)))
motor.SetAngleFunction(chrono.ChFunctionRamp(0, rad_s))
sys.Add(motor)

# FEA mesh
fea_mesh = fea.ChMesh()
sys.Add(fea_mesh)

sys.SetGravitationalAcceleration(chrono.VNULL);
fea_mesh.SetAutomaticGravity(False)

# FEA beams
Izz = (1.0 / 12.0) * beam_wz * beam_wy**3
Iyy = (1.0 / 12.0) * beam_wy * beam_wz**3
damping_coeffs = fea.DampingCoefficients()
damping_coeffs.bt = 0.001
damping_coeffs.bx = 0.001
damping_coeffs.by = 0.001
damping_coeffs.bz = 0.001
section = fea.ChBeamSectionTimoshenkoAdvancedGeneric(
    beam_Young * beam_wy * beam_wz, (Izz + Iyy) * beam_Young * 0.3, Iyy * beam_Young, Izz * beam_Young,
    beam_Young * 0.3 * beam_wy * beam_wz, beam_Young * 0.3 * beam_wy * beam_wz, damping_coeffs, 0, 0, 0, 0, 0,
    beam_density * beam_wy * beam_wz, beam_density * Iyy, beam_density * Izz, 0, 0, 0, 0)

section.SetDrawShape(fea.ChBeamSectionShapeRectangular(beam_wy, beam_wz))

tapered_section = fea.ChBeamSectionTaperedTimoshenkoAdvancedGeneric()
tapered_section.SetSectionA(section)
tapered_section.SetSectionB(section)

builder = fea.ChBuilderBeamTaperedTimoshenko()
builder.BuildBeam(fea_mesh,                            # contining mesh
                  tapered_section,                     # beam section
                  6,                                   # number of sections (spans)
                  chrono.ChVector3d(0, beam_Rmin, 1),  # beam start point
                  chrono.ChVector3d(0, beam_Rmax, 1),  # beam end point
                  chrono.ChVector3d(0, 0, 1)           # up direction of beam section
)

nodes = builder.GetLastBeamNodes()

# Connect root of blade to the hub (use a motor, but with zero speed)
root_motor = chrono.ChLinkMotorRotationAngle()
root_motor.Initialize(nodes.front(), hub, chrono.ChFramed(chrono.ChVector3d(0, 0.5, 1), chrono.QuatFromAngleX(chrono.CH_PI_2)))
root_motor.SetMotorFunction(chrono.ChFunctionConst(0))
sys.Add(root_motor)

# Apply tip force (in absolute frame)
nodes.back().SetForce(tip_abs_force)

# FEA mesh visualization
vis_beam_1 = chrono.ChVisualShapeFEA()
vis_beam_1.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ELEM_BEAM_TX)
vis_beam_1.SetColormapRange(-0.001, 600)
vis_beam_1.SetSmoothFaces(True)
vis_beam_1.SetWireframe(False)
fea_mesh.AddVisualShapeFEA(vis_beam_1)

vis_beam_2 = chrono.ChVisualShapeFEA()
vis_beam_2.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_CSYS)
vis_beam_2.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
vis_beam_2.SetSymbolsThickness(0.2)
vis_beam_2.SetSymbolsScale(0.1)
vis_beam_2.SetZbufferHide(False)
fea_mesh.AddVisualShapeFEA(vis_beam_2)

# Irrlicht run-time visualization
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('FEA Timoshenko beams')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(0, 0, 15.0))
vis.AddTypicalLights()

# Use MKL Pardiso solver
solver = mkl.ChSolverPardisoMKL()
sys.SetSolver(solver)

# Use HHT integrator
integrator = chrono.ChTimestepperHHT(sys)
integrator.SetStepControl(False)
sys.SetTimestepper(integrator)

# Static analysis
class StaticsIterationCallback (chrono.ChStaticNonLinearRheonomicAnalysis_IterationCallback):
    def __init__(self, nodes, rad_s):
        super().__init__()
        self.nodes = nodes
        self.rad_s = rad_s

    def OnIterationBegin(self, load_scaling, iteration_n, analysis):
        for i in range(len(self.nodes)):
            pos = self.nodes[i].GetPos()
            self.nodes[i].SetPosDt(chrono.ChVector3d(-pos.y * self.rad_s, 0, 0))
            self.nodes[i].SetPosDt2(chrono.ChVector3d(0, -pos.y * self.rad_s**2, 0))

static_analysis = chrono.ChStaticNonLinearRheonomicAnalysis()
static_analysis.SetMaxIterations(25)
static_analysis.SetVerbose(True)
statics_callback = StaticsIterationCallback(nodes, rad_s)
static_analysis.SetCallbackIterationBegin(statics_callback)

# Perform nonlinear static analysis
sys.DoStaticAnalysis(static_analysis);

# Dynamic simulation loop
realtime_timer = chrono.ChRealtimeStepTimer()
time_step = 0.01
while vis.Run():
    vis.BeginScene()
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(time_step)
    realtime_timer.Spin(time_step)
