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


import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math

print ("Example: demonstration of using friction models")

# The path to the Chrono data directory containing various assets (meshes, textures, data files)
# is automatically set, relative to the default location of this demo.
# If running from a different directory, you must change the path to the data directory with: 
# chrono.SetChronoDataPath('../../../../data/')


# Helper class to define a cylindrical shape
class MyObstacle:
    def __init__(self, r, pos):
        self.radius = r
        self.center = pos
    def AddVisualization(self, body):
        cyl = chrono.ChVisualShapeCylinder(self.radius, 1.1)
        cyl.SetColor(chrono.ChColor(0.6, 0.3, 0.0))
        body.AddVisualShape(cyl, chrono.ChFramed(self.center + chrono.ChVector3d(0, 0.55, 0),
                                                 chrono.QuatFromAngleX(chrono.CH_PI_2)))

# Custom collision detection callback class
class MyCustomCollisionDetection(chrono.CustomCollisionCallback):
    def __init__(self, ball, ground,
                 ball_mat, obst_mat,
                 ball_radius, obstacle):
        super().__init__()
        self.m_ball = ball
        self.m_ground = ground
        self.m_ball_mat = ball_mat
        self.m_obst_mat = obst_mat
        self.m_ball_radius = ball_radius
        self.m_obst_radius = obstacle.radius
        self.m_obst_center = obstacle.center
        
    def OnCustomCollision(self, sys):
        # super().OnCustomCollision(sys)
        r_sum = self.m_ball_radius + self.m_obst_radius

        # Get current ball position and project on horizontal plane.
        b_pos = self.m_ball.GetPos()
        b_center = chrono.ChVector3d(b_pos.x, 0.0, b_pos.z)

        # Check collision with obstacle (working in the horizontal plane).
        o_center = chrono.ChVector3d(self.m_obst_center.x, 0.0, self.m_obst_center.z)
        delta = o_center - b_center
        # Get the squared euclidean norm
        dist2 = delta.Length2()

        if dist2 >= r_sum * r_sum:
            return

        # Find collision points on the ball and obstacle and the contact normal.
        dist = math.sqrt(dist2)
        normal = delta / dist        
        pt_ball = b_center + normal * self.m_ball_radius
        pt_obst = o_center - normal * self.m_obst_radius

        # Populate the collision info object (express all vectors in 3D).
        # We pass null pointers to collision shapes.
        contact = chrono.ChCollisionInfo()
        contact.modelA = self.m_ball.GetCollisionModel()
        contact.modelB = self.m_ground.GetCollisionModel()
        contact.shapeA = None
        contact.shapeB = None
        contact.vN = chrono.ChVector3d(normal.x, 0.0, normal.z)
        contact.vpA = chrono.ChVector3d(pt_ball.x, b_pos.y, pt_ball.z)
        contact.vpB = chrono.ChVector3d(pt_obst.x, b_pos.y, pt_obst.z)
        contact.distance = dist - r_sum

        sys.GetContactContainer().AddContact(contact, self.m_ball_mat, self.m_obst_mat)


# ---------------------------------------------------------------------
#
#  Create the simulation sys and add items
#

# Change use_NSC to specify different contact method
use_NSC = 0

ball_radius = 0.5
obst_radius = 2.0
obst_center = chrono.ChVector3d(2.9, 0, 2.9)
obstacle = MyObstacle(obst_radius, obst_center)

# Create the sys and the various contact materials
if use_NSC:
    sys = chrono.ChSystemNSC()
    g_mat = chrono.ChContactMaterialNSC()
    g_mat.SetRestitution(0.9)
    g_mat.SetFriction(0.4)
    b_mat = chrono.ChContactMaterialNSC()
    b_mat.SetRestitution(0.9)
    b_mat.SetFriction(0.5)
    o_mat = chrono.ChContactMaterialNSC()
    o_mat.SetRestitution(0.9)
    o_mat.SetFriction(0.4)

    ground_mat = g_mat
    ball_mat = b_mat
    obst_mat = o_mat

    time_step = 1e-3
    frame_skip = 10

else: # use SMC contact method
    sys = chrono.ChSystemSMC()

    g_mat = chrono.ChContactMaterialSMC()
    g_mat.SetRestitution(0.9)
    g_mat.SetFriction(0.4)
    b_mat = chrono.ChContactMaterialSMC()
    b_mat.SetRestitution(0.9)
    b_mat.SetFriction(0.5)
    o_mat = chrono.ChContactMaterialSMC()
    o_mat.SetRestitution(0.9)
    o_mat.SetFriction(0.4)

    ground_mat = g_mat
    ball_mat = b_mat
    obst_mat = o_mat

    time_step = 1e-4
    frame_skip = 100

sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
sys.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.8, 0))

# Create the ground body with a plate and side walls (both collision and visualization).
ground = chrono.ChBody()
sys.AddBody(ground)
ground.EnableCollision(True)
ground.SetFixed(True)


cshape_1 = chrono.ChCollisionShapeBox(ground_mat, 10.0, 2.0, 10.0)
cshape_2 = chrono.ChCollisionShapeBox(ground_mat, 0.2, 2.0, 10.2)
cshape_3 = chrono.ChCollisionShapeBox(ground_mat, 0.2, 2.0, 10.2)
cshape_4 = chrono.ChCollisionShapeBox(ground_mat, 10.2, 2.0, 0.2)
cshape_5 = chrono.ChCollisionShapeBox(ground_mat, 10.2, 2.0, 0.2)

ground.AddCollisionShape(cshape_1, chrono.ChFramed(chrono.ChVector3d(0, -1, 0), chrono.QUNIT))
ground.AddCollisionShape(cshape_2, chrono.ChFramed(chrono.ChVector3d(-5, 0, 0), chrono.QUNIT))
ground.AddCollisionShape(cshape_3, chrono.ChFramed(chrono.ChVector3d( 5, 0, 0), chrono.QUNIT))
ground.AddCollisionShape(cshape_4, chrono.ChFramed(chrono.ChVector3d(0, 0, -5), chrono.QUNIT))
ground.AddCollisionShape(cshape_5, chrono.ChFramed(chrono.ChVector3d(0, 0,  5), chrono.QUNIT))

ground_vis_mat = chrono.ChVisualMaterial()
ground_vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/blue.png"))

vshape_1 = chrono.ChVisualShapeBox(10, 2, 10)
vshape_1.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_1, chrono.ChFramed(chrono.ChVector3d(0, -1, 0)))

vshape_2 = chrono.ChVisualShapeBox(0.2, 2, 10.2)
vshape_2.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_2, chrono.ChFramed(chrono.ChVector3d(-5, 0, 0)))

vshape_3 = chrono.ChVisualShapeBox(0.2, 2, 10.2)
vshape_3.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_3, chrono.ChFramed(chrono.ChVector3d(5, 0, 0)))

vshape_4 = chrono.ChVisualShapeBox(10.2, 2, 0.2)
vshape_4.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_4, chrono.ChFramed(chrono.ChVector3d(0, 0, -5)))

vshape_5 = chrono.ChVisualShapeBox(10.2, 2, 0.2)
vshape_5.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_5, chrono.ChFramed(chrono.ChVector3d(0, 0, 5)))

# Add obstacle visualization
obstacle.AddVisualization(ground)

# Create the falling ball
ball = chrono.ChBody()
sys.AddBody(ball)
ball.SetMass(10)
comp = 4 * ball_radius * ball_radius
ball.SetInertiaXX(chrono.ChVector3d(comp, comp, comp))
ball.SetPos(chrono.ChVector3d(-3, 1.2 * ball_radius, -3))
ball.SetPosDt(chrono.ChVector3d(5, 0, 5))
ball.EnableCollision(True)

ball_ct_shape = chrono.ChCollisionShapeSphere(ball_mat, ball_radius)
ball.AddCollisionShape(ball_ct_shape)

vshape_s = chrono.ChVisualShapeSphere(ball_radius)
vshape_s.SetTexture(chrono.GetChronoDataFile("textures/bluewhite.png"))
ball.AddVisualShape(vshape_s)

# Create a custom collision detection callback object and register it with the sys
my_collision = MyCustomCollisionDetection(ball, ground, ball_mat, obst_mat, ball_radius, obstacle)
sys.RegisterCustomCollisionCallback(my_collision)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the sys
#

vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(sys)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Custom contact demo')
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_chrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(8, 8, -6))
vis.AddTypicalLights()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

frame = 0
while vis.Run():
    if frame % 100 == 0:
        vis.BeginScene() 
        vis.Render()
        vis.EndScene()
    sys.DoStepDynamics(1e-4)
    frame += 1




