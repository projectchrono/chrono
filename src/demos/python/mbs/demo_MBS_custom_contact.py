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
    def GetVisualization(self):
        cyl = chrono.ChCylinderShape()
        cyl.GetCylinderGeometry().rad = self.radius
        cyl.GetCylinderGeometry().p1 = self.center + chrono.ChVectorD(0, 0, 0)
        cyl.GetCylinderGeometry().p2 = self.center + chrono.ChVectorD(0, 1.1, 0)
        cyl.SetColor(chrono.ChColor(0.6, 0.3, 0.0))
        return cyl

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
        b_center = chrono.ChVectorD(b_pos.x, 0.0, b_pos.z)

        # Check collision with obstacle (working in the horizontal plane).
        o_center = chrono.ChVectorD(self.m_obst_center.x, 0.0, self.m_obst_center.z)
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
        contact.vN = chrono.ChVectorD(normal.x, 0.0, normal.z)
        contact.vpA = chrono.ChVectorD(pt_ball.x, b_pos.y, pt_ball.z)
        contact.vpB = chrono.ChVectorD(pt_obst.x, b_pos.y, pt_obst.z)
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
obst_center = chrono.ChVectorD(2.9, 0, 2.9)
obstacle = MyObstacle(obst_radius, obst_center)

# Create the sys and the various contact materials
if use_NSC:
    sys = chrono.ChSystemNSC()
    g_mat = chrono.ChMaterialSurfaceNSC()
    g_mat.SetRestitution(0.9)
    g_mat.SetFriction(0.4)
    b_mat = chrono.ChMaterialSurfaceNSC()
    b_mat.SetRestitution(0.9)
    b_mat.SetFriction(0.5)
    o_mat = chrono.ChMaterialSurfaceNSC()
    o_mat.SetRestitution(0.9)
    o_mat.SetFriction(0.4)

    ground_mat = g_mat
    ball_mat = b_mat
    obst_mat = o_mat

    time_step = 1e-3
    frame_skip = 10

else: # use SMC contact method
    sys = chrono.ChSystemSMC()

    g_mat = chrono.ChMaterialSurfaceSMC()
    g_mat.SetRestitution(0.9)
    g_mat.SetFriction(0.4)
    b_mat = chrono.ChMaterialSurfaceSMC()
    b_mat.SetRestitution(0.9)
    b_mat.SetFriction(0.5)
    o_mat = chrono.ChMaterialSurfaceSMC()
    o_mat.SetRestitution(0.9)
    o_mat.SetFriction(0.4)

    ground_mat = g_mat
    ball_mat = b_mat
    obst_mat = o_mat

    time_step = 1e-4
    frame_skip = 100

sys.Set_G_acc(chrono.ChVectorD(0, -9.8, 0))

# Create the ground body with a plate and side walls (both collision and visualization).
ground = chrono.ChBody()
sys.AddBody(ground)
ground.SetCollide(True)
ground.SetBodyFixed(True)

ground.GetCollisionModel().ClearModel()
ground.GetCollisionModel().AddBox(ground_mat, 5.0, 1.0, 5.0, chrono.ChVectorD(0, -1, 0))
ground.GetCollisionModel().AddBox(ground_mat, 0.1, 1.0, 5.1, chrono.ChVectorD(-5, 0, 0))
ground.GetCollisionModel().AddBox(ground_mat, 0.1, 1.0, 5.1, chrono.ChVectorD( 5, 0, 0))
ground.GetCollisionModel().AddBox(ground_mat, 5.1, 1.0, 0.1, chrono.ChVectorD(0, 0, -5))
ground.GetCollisionModel().AddBox(ground_mat, 5.1, 1.0, 0.1, chrono.ChVectorD(0, 1,  5))
ground.GetCollisionModel().BuildModel()

ground_vis_mat = chrono.ChVisualMaterial()
ground_vis_mat.SetKdTexture(chrono.GetChronoDataFile("textures/blue.png"))

vshape_1 = chrono.ChBoxShape()
vshape_1.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 2, 10))
vshape_1.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_1, chrono.ChFrameD(chrono.ChVectorD(0, -1, 0)))

vshape_2 = chrono.ChBoxShape()
vshape_2.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 2, 10.2))
vshape_2.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_2, chrono.ChFrameD(chrono.ChVectorD(-5, 0, 0)))

vshape_3 = chrono.ChBoxShape()
vshape_3.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 2, 10.2))
vshape_3.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_3, chrono.ChFrameD(chrono.ChVectorD(5, 0, 0)))

vshape_4 = chrono.ChBoxShape()
vshape_4.GetBoxGeometry().SetLengths(chrono.ChVectorD(10.2, 2, 0.2))
vshape_4.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_4, chrono.ChFrameD(chrono.ChVectorD(0, 0, -5)))

vshape_5 = chrono.ChBoxShape()
vshape_5.GetBoxGeometry().SetLengths(chrono.ChVectorD(10.2, 2, 0.2))
vshape_5.SetMaterial(0, ground_vis_mat)
ground.AddVisualShape(vshape_5, chrono.ChFrameD(chrono.ChVectorD(0, 0, 5)))

# Add obstacle visualization
ground.AddVisualShape(obstacle.GetVisualization())

# Create the falling ball
ball = chrono.ChBody()
sys.AddBody(ball)
ball.SetMass(10)
comp = 4 * ball_radius * ball_radius
ball.SetInertiaXX(chrono.ChVectorD(comp, comp, comp))
ball.SetPos(chrono.ChVectorD(-3, 1.2 * ball_radius, -3))
ball.SetPos_dt(chrono.ChVectorD(5, 0, 5))
ball.SetCollide(True)

ball.GetCollisionModel().ClearModel()
ball.GetCollisionModel().AddSphere(ball_mat, ball_radius)
ball.GetCollisionModel().BuildModel()

vshape_s = chrono.ChSphereShape()
vshape_s.GetSphereGeometry().rad = ball_radius
vshape_s.GetSphereGeometry().Pos = ball.GetPos()
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
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVectorD(8, 8, -6))
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




