# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2019 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================


import math
import pychrono as chrono
import pychrono.irrlicht as chronoirr

print ("Example: study the associative effect of friction.");


# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#


sys      = chrono.ChSystemNSC()

# Global collision tolerances
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.01)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.005)
#chrono.ChCollisionSystemBullet.SetContactBreakingThreshold(0.01)

# Create a fixed rigid body

phi   = 40 * chrono.CH_C_DEG_TO_RAD;
rad_sph = 0.1
alpha = 45 * chrono.CH_C_DEG_TO_RAD
beta  = alpha - 0 * chrono.CH_C_DEG_TO_RAD
thick = 0.2
fixed_L = True

friction= math.tan(phi)
print(friction)
brick_material = chrono.ChMaterialSurfaceNSC()
brick_material.SetFriction(friction)
brick_material.SetDampingF(0.00000)
brick_material.SetCompliance (1e-9)
brick_material.SetComplianceT(1e-9)

L_material = chrono.ChMaterialSurfaceNSC()
L_material.SetFriction(0)


S0 = chrono.ChVectorD(0,0,0)
Sr = chrono.ChVectorD( rad_sph * math.sin(alpha), -rad_sph * math.cos(alpha), 0)
Sl = chrono.ChVectorD(-rad_sph * math.sin(beta),   rad_sph * math.cos(beta), 0)
Dr = chrono.ChVectorD(  math.cos(alpha),  math.sin(alpha), 0)
Dl = chrono.ChVectorD(  math.cos(beta),   math.sin(beta), 0)
Dz = chrono.ChVectorD(0,0, thick)



if False:
    mbodyS = chrono.ChBodyEasySphere(0.1,1000,True,True,brick_material)
    mbodyS.SetPos( S0 )
    sys.Add(mbodyS)
else:
    S1 = Sl + Dl * 0.2;
    S2 = Sl - Dl * 0.2;
    S3 = Sr + Dr * 0.2;
    S4 = Sr - Dr * 0.2;
    Dzz = chrono.ChVectorD(0,0, (thick*0.6))
    pointsS = chrono.vector_ChVectorD([S1+Dzz, S2+Dzz, S3+Dzz, S4+Dzz, S1-Dzz, S2-Dzz, S3-Dzz, S4-Dzz])

    mbodyS = chrono.ChBodyEasyConvexHullAuxRef(pointsS, 1000,True,True,brick_material)
    sys.Add(mbodyS)

L1 = Sl + Dl * 0.5;
L2 = Sl - Dl * 0.5;
L3 = chrono.ChVectorD(-2,   L2.y, 0);
L4 = chrono.ChVectorD(-2,   L1.y, 0);
pointsL = chrono.vector_ChVectorD([L1+Dz, L2+Dz, L3+Dz, L4+Dz, L1-Dz, L2-Dz, L3-Dz, L4-Dz])

mbodyL = chrono.ChBodyEasyConvexHullAuxRef(pointsL, 1000,True,True,L_material)
mbodyL.SetBodyFixed(fixed_L)
sys.Add(mbodyL)


R1 = Sr + Dr * 0.5;
R2 = Sr - Dr * 0.5;
R3 = chrono.ChVectorD(1,   R2.y, 0);
R4 = chrono.ChVectorD(1,   R1.y, 0);
pointsR = chrono.vector_ChVectorD([R1+Dz, R2+Dz, R3+Dz, R4+Dz, R1-Dz, R2-Dz, R3-Dz, R4-Dz])

mbodyR = chrono.ChBodyEasyConvexHullAuxRef(pointsR, 1000,True,True,brick_material)
mbodyR.SetBodyFixed(True)
sys.Add(mbodyR)


if not(fixed_L):
    mbodyG = chrono.ChBodyEasyBox(1,0.5 , thick*2.2, 1000,True,True,brick_material)
    mbodyG.SetPos( chrono.ChVectorD(-1, L2.y-0.5/2, 0 ) )
    mbodyG.SetBodyFixed(True)
    sys.Add(mbodyG)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
sys.SetVisualSystem(vis)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Test')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(0.6,0.6,0.8))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

# Integration settings

solver = chrono.ChSolverBB()
sys.SetSolver(solver)
solver.SetMaxIterations(500)
solver.EnableWarmStart(True);

sys.SetMaxPenetrationRecoverySpeed(10.01);
sys.SetMinBounceSpeed(10);

while vis.Run():
    vis.BeginScene() 
    vis.DrawAll()
    vis.EndScene()
    sys.DoStepDynamics(5e-4)





