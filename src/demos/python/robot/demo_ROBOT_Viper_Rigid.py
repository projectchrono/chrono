# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2021 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demo to show Viper Rover operated on Rigid Terrain
#
# =============================================================================

import os
import math
import numpy as np
import pychrono as chrono
import pychrono.robot as viper
try:
   from pychrono import irrlicht as chronoirr
except:
   print('Could not import ChronoIrrlicht')

# Chreate Chrono system
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, -9.81, 0))
chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

# Create ground body
ground_mat = chrono.ChMaterialSurfaceNSC()
ground = chrono.ChBodyEasyBox(20, 1, 20, 1000, True, True, ground_mat)
ground.SetPos(chrono.ChVectorD(0, -1, 0))
ground.SetBodyFixed(True)
system.Add(ground)

texture = chrono.ChTexture()
texture.SetTextureFilename(chrono.GetChronoDataFile("concrete.jpg"))
ground.AddAsset(texture)

# Create Viper rover
viper = viper.ViperRover(system, chrono.ChVectorD(0, -0.2, 0), chrono.Q_from_AngX(-chrono.CH_C_PI_2))
viper.Initialize()

# Create run-time visualization
application = chronoirr.ChIrrApp(system, "Viper rover - Rigid terrain", chronoirr.dimension2du(1280, 720))
application.AddTypicalLogo(chrono.GetChronoDataPath() + 'logo_pychrono_alpha.png')
application.AddTypicalSky()
application.AddTypicalCamera(chronoirr.vector3df(0, 2, -2), chronoirr.vector3df(0, 0, 0))
application.AddTypicalLights(chronoirr.vector3df(100, 100, 100), chronoirr.vector3df(100, -100, 80))
application.AddLightWithShadow(chronoirr.vector3df(1.5, 5.5, -2.5), chronoirr.vector3df(0, 0, 0), 3, 4, 10, 40, 512)

application.AssetBindAll()
application.AssetUpdateAll()
application.AddShadowAll()

application.SetTimestep(0.0005)

# Simulation loop
while (application.GetDevice().run()) :
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
    application.DrawAll()
    application.DoStep()
    application.EndScene()
