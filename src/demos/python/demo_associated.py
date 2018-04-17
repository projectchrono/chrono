#-------------------------------------------------------------------------------
# Name:        modulo1
# Purpose:
#
# Author:      tasora
#
# Created:     30/11/2017
# Copyright:   (c) tasora 2017
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

def main():
    pass

if __name__ == '__main__':
    main()


import os
import math
import ChronoEngine_python_core as chrono
import ChronoEngine_python_postprocess as postprocess
import ChronoEngine_python_irrlicht as chronoirr

print ("Example: create a system and visualize it in realtime 3D");


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
# brick_material.SetRollingFriction(rollfrict_param)
# brick_material.SetSpinningFriction(0)
# brick_material.SetComplianceRolling(0.0000001)
# brick_material.SetComplianceSpinning(0.0000001)

L_material = chrono.ChMaterialSurfaceNSC()
L_material.SetFriction(0)
# brick_material.SetDampingF(0.2)
# brick_material.SetCompliance (0.0000001)
# brick_material.SetComplianceT(0.0000001)
# brick_material.SetRollingFriction(rollfrict_param)
# brick_material.SetSpinningFriction(0)
# brick_material.SetComplianceRolling(0.0000001)
# brick_material.SetComplianceSpinning(0.0000001)


S0 = chrono.ChVectorD(0,0,0)
Sr = chrono.ChVectorD( rad_sph * math.sin(alpha), -rad_sph * math.cos(alpha), 0)
Sl = chrono.ChVectorD(-rad_sph * math.sin(beta),   rad_sph * math.cos(beta), 0)
Dr = chrono.ChVectorD(  math.cos(alpha),  math.sin(alpha), 0)
Dl = chrono.ChVectorD(  math.cos(beta),   math.sin(beta), 0)
Dz = chrono.ChVectorD(0,0, thick)



if False:
    mbodyS = chrono.ChBodyEasySphere(0.1,1000,True,True)
    mbodyS.SetPos( S0 )
    mbodyS.SetMaterialSurface(brick_material)
    mysystem.Add(mbodyS)
else:
    S1 = Sl + Dl * 0.2;
    S2 = Sl - Dl * 0.2;
    S3 = Sr + Dr * 0.2;
    S4 = Sr - Dr * 0.2;
    Dzz = chrono.ChVectorD(0,0, (thick*0.6))
    pointsS = chrono.vector_ChVectorD([S1+Dzz, S2+Dzz, S3+Dzz, S4+Dzz, S1-Dzz, S2-Dzz, S3-Dzz, S4-Dzz])

    mbodyS = chrono.ChBodyEasyConvexHullAuxRef(pointsS, 1000,True,True)
    mbodyS.SetMaterialSurface(brick_material)
    mysystem.Add(mbodyS)

L1 = Sl + Dl * 0.5;
L2 = Sl - Dl * 0.5;
L3 = chrono.ChVectorD(-2,   L2.y, 0);
L4 = chrono.ChVectorD(-2,   L1.y, 0);
pointsL = chrono.vector_ChVectorD([L1+Dz, L2+Dz, L3+Dz, L4+Dz, L1-Dz, L2-Dz, L3-Dz, L4-Dz])

mbodyL = chrono.ChBodyEasyConvexHullAuxRef(pointsL, 1000,True,True)
mbodyL.SetBodyFixed(fixed_L)
mbodyL.SetMaterialSurface(L_material)
mysystem.Add(mbodyL)


R1 = Sr + Dr * 0.5;
R2 = Sr - Dr * 0.5;
R3 = chrono.ChVectorD(1,   R2.y, 0);
R4 = chrono.ChVectorD(1,   R1.y, 0);
pointsR = chrono.vector_ChVectorD([R1+Dz, R2+Dz, R3+Dz, R4+Dz, R1-Dz, R2-Dz, R3-Dz, R4-Dz])

mbodyR = chrono.ChBodyEasyConvexHullAuxRef(pointsR, 1000,True,True)
mbodyR.SetBodyFixed(True)
mbodyR.SetMaterialSurface(brick_material)
mysystem.Add(mbodyR)


if not(fixed_L):
    mbodyG = chrono.ChBodyEasyBox(1,0.5 , thick*2.2, 1000,True,True)
    mbodyG.SetPos( chrono.ChVectorD(-1, L2.y-0.5/2, 0 ) )
    mbodyG.SetBodyFixed(True)
    mbodyG.SetMaterialSurface(brick_material)
    mysystem.Add(mbodyG)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'Test', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky('../../../data/skybox/')
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

mysystem.SetMaxPenetrationRecoverySpeed(10.01);
mysystem.SetMinBounceSpeed(10);
mysystem.SetMaxItersSolverSpeed(500);
mysystem.SetSolverWarmStarting(True);
mysystem.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN);

myapplication.SetTimestep(0.0005)


while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()





