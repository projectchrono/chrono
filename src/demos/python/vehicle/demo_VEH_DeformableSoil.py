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
# Authors: Radu Serban
# =============================================================================
# Demo of the SCM semi-empirical model for deformable soil
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh

import math

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
#chrono.SetChronoDataPath('path/to/data')

# If true, use provided callback to change soil properties based on location
var_params = True

class MySoilParams (veh.SoilParametersCallback):
    def __init__(self):
        veh.SoilParametersCallback.__init__(self)
    def Set(self, loc, Kphi, Kc, n, coh, mu_angle, shear, K, R):
        Kphi_ = veh.doublep_value(Kphi)
        Kc_ = veh.doublep_value(Kc)
        n_ = veh.doublep_value(n)
        coh_ = veh.doublep_value(coh)
        mu_angle_ = veh.doublep_value(mu_angle)
        shear_ = veh.doublep_value(shear)
        K_ = veh.doublep_value(K)
        R_ = veh.doublep_value(R)
        if loc.y > 0 :
            Kphi_ = 0.2e6
            Kc_ = 0
            n_ = 1.1
            coh_ = 0
            mu_angle_ = 30
            shear_ = 0.01
            K_ = 4e7
            R_ = 3e4
        else:
            Kphi_ = 5301e3
            Kc_ = 102e3
            n_ = 0.793
            coh_ = 1.3e3
            mu_angle_ = 31.1
            shear_ = 1.2e-2
            K_ = 4e8
            R_ = 3e4
        veh.doublep_assign(Kphi, Kphi_)
        veh.doublep_assign(Kc, Kc_)
        veh.doublep_assign(n, n_)
        veh.doublep_assign(coh, coh_)
        veh.doublep_assign(mu_angle, mu_angle_)
        veh.doublep_assign(shear, shear_)
        veh.doublep_assign(K, K_)
        veh.doublep_assign(R, R_)
        

# Global parameters for tire
tire_rad = 0.8
tire_vel_z0 = -3
tire_center = chrono.ChVector3d(0, 0.02 + tire_rad, -1.5)
tire_w0 = tire_vel_z0 / tire_rad

# ----------------------------
# Create the mechanical system
# ----------------------------

sys = chrono.ChSystemSMC()
sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# Create the ground
ground = chrono.ChBody()
ground.SetFixed(True)
sys.Add(ground)

# Create the rigid body with contact mesh
body = chrono.ChBody()
sys.Add(body)
body.SetMass(500)
body.SetInertiaXX(chrono.ChVector3d(20, 20, 20))
body.SetPos(tire_center + chrono.ChVector3d(0, 0.3, 0))

# Load mesh
mesh = chrono.ChTriangleMeshConnected()
mesh.LoadWavefrontMesh(chrono.GetChronoDataFile('models/tractor_wheel/tractor_wheel.obj'))

# Set visualization assets
vis_shape = chrono.ChVisualShapeTriangleMesh()
vis_shape.SetMesh(mesh)
vis_shape.SetColor(chrono.ChColor(0.3, 0.3, 0.3))
body.AddVisualShape(vis_shape)

# Set collision shape
material = chrono.ChContactMaterialSMC()

body_ct_shape = chrono.ChCollisionShapeTriangleMesh(material, # contact material
                                                    mesh,     # the mesh 
                                                    False,    # is it static?
                                                    False,    # is it convex?
                                                    0.01)     # "thickness" for increased robustness
body.AddCollisionShape(body_ct_shape)
body.EnableCollision(True)

# Create motor
motor = chrono.ChLinkMotorRotationAngle()
motor.SetSpindleConstraint(chrono.ChLinkMotorRotation.SpindleConstraint_OLDHAM)
motor.SetAngleFunction(chrono.ChFunctionRamp(0, math.pi / 4))
motor.Initialize(body, ground, chrono.ChFramed(tire_center, chrono.QuatFromAngleY(math.pi/2)))
sys.Add(motor)

# ------------------------
# Create SCM terrain patch
# ------------------------

# Note that SCMTerrain uses a default ISO reference frame (Z up). Since the mechanism is modeled here in
# a Y-up global frame, we rotate the terrain plane by -90 degrees about the X axis.
terrain = veh.SCMTerrain(sys)
terrain.SetPlane(chrono.ChCoordsysd(chrono.ChVector3d(0,0.2,0), chrono.QuatFromAngleX(-math.pi/2)))
terrain.Initialize(2.0, 6.0, 0.04)

my_params = MySoilParams()
if var_params:
    # Location-dependent soil properties
    terrain.RegisterSoilParametersCallback(my_params)
else :
    # Constant soil properties
    terrain.SetSoilParameters(0.2e6,  # Bekker Kphi
                               0,      # Bekker Kc
                               1.1,    # Bekker n exponent
                               0,      # Mohr cohesive limit (Pa)
                               30,     # Mohr friction limit (degrees)
                               0.01,   # Janosi shear coefficient (m)
                               4e7,    # Elastic stiffness (Pa/m), before plastic yield, must be > Kphi
                               3e4     # Damping (Pa s/m), proportional to negative vertical speed (optional)
    )

# Set terrain visualization mode
terrain.SetPlotType(veh.SCMTerrain.PLOT_PRESSURE, 0, 30000.2)

# ------------------------------------------
# Create the Irrlicht run-time visualization
# ------------------------------------------

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1280,720)
vis.SetWindowTitle('Deformable soil')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2.0,1.4,0.0), chrono.ChVector3d(0,tire_rad,0))
vis.AddTypicalLights()

# ------------------
# Run the simulation
# ------------------

while vis.Run() :
    vis.BeginScene()
    vis.GetSceneManager().getActiveCamera().setTarget(chronoirr.vector3dfCH(body.GetPos()))
    vis.Render()
    vis.EndScene()
    sys.DoStepDynamics(0.002)
