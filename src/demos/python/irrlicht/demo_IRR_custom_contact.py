#------------------------------------------------------------------------------
# Name:        pychrono example
# Purpose:
#
# Author:      Lijing Yang
#
# Created:     6/12/2020
# Copyright:   (c) ProjectChrono 2019
#------------------------------------------------------------------------------


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
        level = chrono.ChAssetLevel()
        cyl = chrono.ChCylinderShape()
        cyl.GetCylinderGeometry().rad = self.radius
        cyl.GetCylinderGeometry().p1 = self.center + chrono.ChVectorD(0, 0, 0)
        cyl.GetCylinderGeometry().p2 = self.center + chrono.ChVectorD(0, 1.1, 0)
        level.AddAsset(cyl)
        level.AddAsset(chrono.ChColorAsset(0.6, 0.3, 0.0))
        return level

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
#  Create the simulation system and add items
#

# Change use_NSC to specify different contact method
use_NSC = 0

ball_radius = 0.5
obst_radius = 2.0
obst_center = chrono.ChVectorD(2.9, 0, 2.9)
obstacle = MyObstacle(obst_radius, obst_center)

# Create the system and the various contact materials
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

vshape_1 = chrono.ChBoxShape()
vshape_1.GetBoxGeometry().SetLengths(chrono.ChVectorD(10, 2, 10))
vshape_1.GetBoxGeometry().Pos = chrono.ChVectorD(0, -1, 0)
ground.AddAsset(vshape_1)

vshape_2 = chrono.ChBoxShape()
vshape_2.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 2, 10.2))
vshape_2.GetBoxGeometry().Pos = chrono.ChVectorD(-5, 0, 0)
ground.AddAsset(vshape_2)

vshape_3 = chrono.ChBoxShape()
vshape_3.GetBoxGeometry().SetLengths(chrono.ChVectorD(0.2, 2, 10.2))
vshape_3.GetBoxGeometry().Pos = chrono.ChVectorD(5, 0, 0)
ground.AddAsset(vshape_3)

vshape_4 = chrono.ChBoxShape()
vshape_4.GetBoxGeometry().SetLengths(chrono.ChVectorD(10.2, 2, 0.2))
vshape_4.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, -5)
ground.AddAsset(vshape_4)

vshape_5 = chrono.ChBoxShape()
vshape_5.GetBoxGeometry().SetLengths(chrono.ChVectorD(10.2, 2, 0.2))
vshape_5.GetBoxGeometry().Pos = chrono.ChVectorD(0, 0, 5)
ground.AddAsset(vshape_5)

ground.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("blu.png")))

# Add obstacle visualization (in a separate level with a different color).
ground.AddAsset(obstacle.GetVisualization())

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
ball.AddAsset(vshape_s)
ball.AddAsset(chrono.ChTexture(chrono.GetChronoDataFile("bluwhite.png")))

# Create a custom collision detection callback object and register it with the system
my_collision = MyCustomCollisionDetection(ball, ground, ball_mat, obst_mat, ball_radius, obstacle)
sys.RegisterCustomCollisionCallback(my_collision)


# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(sys, 'PyChrono example: Custom contact', chronoirr.dimension2du(1024,768))

myapplication.AddTypicalSky()
myapplication.AddTypicalLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
myapplication.AddTypicalCamera(chronoirr.vector3df(8, 8, -6))
myapplication.AddTypicalLights()

# ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
# If you need a finer control on which item really needs a visualization proxy in
# Irrlicht, just use application.AssetBind(myitem) on a per-item basis.

myapplication.AssetBindAll()

# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll()


# ---------------------------------------------------------------------
#
#  Run the simulation
#

myapplication.SetTimestep(1e-4)
myapplication.SetTryRealtime(True)

frame = 0
while(myapplication.GetDevice().run()):
    if frame % 100 == 0:
        myapplication.BeginScene()
        myapplication.DrawAll()
        myapplication.EndScene()
    myapplication.DoStep()
    frame += 1




