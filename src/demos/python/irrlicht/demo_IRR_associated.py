#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Alessandro Tasora
#
# Created:     1/01/2019
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


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
#  Create the simulation system and add items
#


mysystem      = chrono.ChSystemNSC()

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
    mysystem.Add(mbodyS)
else:
    S1 = Sl + Dl * 0.2;
    S2 = Sl - Dl * 0.2;
    S3 = Sr + Dr * 0.2;
    S4 = Sr - Dr * 0.2;
    Dzz = chrono.ChVectorD(0,0, (thick*0.6))
    pointsS = chrono.vector_ChVectorD([S1+Dzz, S2+Dzz, S3+Dzz, S4+Dzz, S1-Dzz, S2-Dzz, S3-Dzz, S4-Dzz])

    mbodyS = chrono.ChBodyEasyConvexHullAuxRef(pointsS, 1000,True,True,brick_material)
    mysystem.Add(mbodyS)

L1 = Sl + Dl * 0.5;
L2 = Sl - Dl * 0.5;
L3 = chrono.ChVectorD(-2,   L2.y, 0);
L4 = chrono.ChVectorD(-2,   L1.y, 0);
pointsL = chrono.vector_ChVectorD([L1+Dz, L2+Dz, L3+Dz, L4+Dz, L1-Dz, L2-Dz, L3-Dz, L4-Dz])

mbodyL = chrono.ChBodyEasyConvexHullAuxRef(pointsL, 1000,True,True,L_material)
mbodyL.SetBodyFixed(fixed_L)
mysystem.Add(mbodyL)


R1 = Sr + Dr * 0.5;
R2 = Sr - Dr * 0.5;
R3 = chrono.ChVectorD(1,   R2.y, 0);
R4 = chrono.ChVectorD(1,   R1.y, 0);
pointsR = chrono.vector_ChVectorD([R1+Dz, R2+Dz, R3+Dz, R4+Dz, R1-Dz, R2-Dz, R3-Dz, R4-Dz])

mbodyR = chrono.ChBodyEasyConvexHullAuxRef(pointsR, 1000,True,True,brick_material)
mbodyR.SetBodyFixed(True)
mysystem.Add(mbodyR)


if not(fixed_L):
    mbodyG = chrono.ChBodyEasyBox(1,0.5 , thick*2.2, 1000,True,True,brick_material)
    mbodyG.SetPos( chrono.ChVectorD(-1, L2.y-0.5/2, 0 ) )
    mbodyG.SetBodyFixed(True)
    mysystem.Add(mbodyG)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'Test', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalSky()
myapplication.AddTypicalCamera(chronoirr.vector3df(0.6,0.6,0.8))
myapplication.AddTypicalLights()

            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			# If you need a finer control on which item really needs a visualization proxy in
			# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

myapplication.AssetBindAll();

			# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll();


# ---------------------------------------------------------------------
#
#  Run the simulation
#

# Integration settings

solver = chrono.ChSolverBB()
mysystem.SetSolver(solver)
solver.SetMaxIterations(500)
solver.EnableWarmStart(True);

mysystem.SetMaxPenetrationRecoverySpeed(10.01);
mysystem.SetMinBounceSpeed(10);

myapplication.SetTimestep(0.0005)


while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()





