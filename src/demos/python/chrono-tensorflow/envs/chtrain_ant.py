#-------------------------------------------------------------------------------
# Load the Chrono::Engine units
import pychrono as chrono
from pychrono import irrlicht as chronoirr
import numpy as np

import math


class Model(object):
   def __init__(self, render):

      self.animate = render
      self.observation_space = np.empty([30,])
      self.action_space = np.zeros([8,])
      self.info =  {}
    # ---------------------------------------------------------------------
    #
    #  Create the simulation system and add items
    #
      self.Xtarg = 1000.0
      self.Ytarg = 0.0
      self.d_old = np.linalg.norm(self.Xtarg + self.Ytarg)
      self.ant_sys = chrono.ChSystemNSC()

    # Set the default outward/inward shape margins for collision detection,
    # this is epecially important for very large or very small objects.
      chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
      chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

    #ant_sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow
      self.ant_sys.SetMaxItersSolverSpeed(70)
      

      self.ant_material = chrono.ChMaterialSurfaceNSC()
      self.ant_material.SetFriction(0.5)
      self.ant_material.SetDampingF(0.2)
      self.ant_material.SetCompliance (0.0005)
      self.ant_material.SetComplianceT(0.0005)

      self.timestep = 0.01
      self.abdomen_x = 0.25
      self.abdomen_y = 0.25
      self.abdomen_z = 0.25

      self.leg_density = 250    # kg/m^3
      self.abdomen_density = 100
      self.abdomen_y0 = 0.4
      self.leg_length = 0.3
      self.leg_radius = 0.04
      self.ankle_angle = 60*(math.pi/180)
      self.ankle_length = 0.4
      self.ankle_radius = 0.04
      self.gain = 30

      self.abdomen_mass = self.abdomen_density * ((4/3)*chrono.CH_C_PI*self.abdomen_x*self.abdomen_y*self.abdomen_z)
      self.abdomen_inertia = chrono.ChVectorD((1/5)*self.abdomen_mass*(pow(self.abdomen_y,2)+pow(self.abdomen_z,2)),(1/5)*self.abdomen_mass*(pow(self.abdomen_x,2)+pow(self.abdomen_z,2)),(1/5)*self.abdomen_mass*(pow(self.abdomen_y,2)+pow(self.abdomen_x,2)))
      self.leg_mass = self.leg_density * self.leg_length * math.pi* pow (self.leg_radius,2)
      self.leg_inertia = chrono.ChVectorD(0.5*self.leg_mass*pow(self.leg_radius,2), (self.leg_mass/12)*(3*pow(self.leg_radius,2)+pow(self.leg_length,2)),(self.leg_mass/12)*(3*pow(self.leg_radius,2)+pow(self.leg_length,2)))
      self.ankle_mass = self.leg_density * self.ankle_length * math.pi* pow (self.ankle_radius,2)
      self.ankle_inertia = chrono.ChVectorD(0.5*self.ankle_mass*pow(self.ankle_radius,2), (self.ankle_mass/12)*(3*pow(self.ankle_radius,2)+pow(self.ankle_length,2)),(self.ankle_mass/12)*(3*pow(self.ankle_radius,2)+pow(self.ankle_length,2)))
      
      self.leg_limit = chrono.ChLinkLimit()
      self.ankle_limit = chrono.ChLinkLimit()
      self.leg_limit.SetRmax(math.pi/9)
      self.leg_limit.SetRmin(-math.pi/9)
      self.ankle_limit.SetRmax(math.pi/9)
      self.ankle_limit.SetRmin(-math.pi/9)
      
      if (self.animate) :
             self.myapplication = chronoirr.ChIrrApp(self.ant_sys)
             self.myapplication.AddShadowAll()
             self.myapplication.SetStepManage(True)
             self.myapplication.SetTimestep(self.timestep)
             self. myapplication.SetTryRealtime(True)  
             self.myapplication.AddTypicalSky('../data/skybox/')
             self.myapplication.AddTypicalCamera(chronoirr.vector3df(0,1.5,0))
             self.myapplication.AddLightWithShadow(chronoirr.vector3df(4,4,0),    # point
                                            chronoirr.vector3df(0,0,0),    # aimpoint
                                            20,                 # radius (power)
                                            1,9,               # near, far
                                            90)                # angle of FOV

   def reset(self):
    
      self.isdone = False
      self.ant_sys.Clear()
      self.body_abdomen = chrono.ChBody()
      self.body_abdomen.SetPos(chrono.ChVectorD(0, self.abdomen_y0, 0 ))
      self.body_abdomen.SetMass(self.abdomen_mass)
      self.body_abdomen.SetInertiaXX(self.abdomen_inertia)
    # set collision surface properties
      self.body_abdomen.SetMaterialSurface(self.ant_material)
      abdomen_ellipsoid = chrono.ChEllipsoid(chrono.ChVectorD(0, 0, 0 ), chrono.ChVectorD(self.abdomen_x, self.abdomen_y, self.abdomen_z ))
      self.abdomen_shape = chrono.ChEllipsoidShape(abdomen_ellipsoid)
      self.body_abdomen.AddAsset(self.abdomen_shape)
      self.body_abdomen.SetMaterialSurface(self.ant_material)
      self.body_abdomen.SetCollide(True)
      self.body_abdomen.GetCollisionModel().ClearModel()
      self.body_abdomen.GetCollisionModel().AddEllipsoid(self.abdomen_x, self.abdomen_y, self.abdomen_z, chrono.ChVectorD(0, 0, 0 ) )
      self.body_abdomen.GetCollisionModel().BuildModel()
      self.ant_sys.Add(self.body_abdomen)
      
      
      leg_ang =  (1/4)*math.pi+(1/2)*math.pi*np.array([0,1,2,3])
      Leg_quat = [chrono.ChQuaternionD() for i in range(len(leg_ang))]
      self.leg_body = [chrono.ChBody() for i in range(len(leg_ang))]
      self.leg_pos= [chrono.ChVectorD() for i in range(len(leg_ang))]
      leg_cyl = chrono.ChCylinder(-chrono.ChVectorD( self.leg_length/2,  0  ,0),chrono.ChVectorD( self.leg_length/2,  0  ,0), self.leg_radius) 
      self.leg_shape = chrono.ChCylinderShape(leg_cyl)
      ankle_cyl = chrono.ChCylinder(-chrono.ChVectorD( self.ankle_length/2,  0  ,0),chrono.ChVectorD( self.ankle_length/2,  0  ,0), self.ankle_radius) 
      self.ankle_shape = chrono.ChCylinderShape(ankle_cyl)
      foot_sphere = chrono.ChSphere(chrono.ChVectorD(self.ankle_length/2, 0, 0 ), self.ankle_radius )
      self.foot_shape = chrono.ChSphereShape(foot_sphere)
      Leg_qa = [ chrono.ChQuaternionD()  for i in range(len(leg_ang))]
      Leg_q = [ chrono.ChQuaternionD()  for i in range(len(leg_ang))]
      z2x_leg = [ chrono.ChQuaternionD() for i in range(len(leg_ang))]
      Leg_rev_pos=[]
      Leg_chordsys = []
      self.legjoint_frame = []
      x_rel = []
      z_rel = []
      self.Leg_rev = [chrono.ChLinkLockRevolute() for i in range(len(leg_ang))]
      self.leg_motor = [chrono.ChLinkMotorRotationTorque() for i in range(len(leg_ang)) ]
      #ankle lists
      anklejoint_chordsys = []
      self.anklejoint_frame = []
      self.ankleCOG_frame = []
      q_ankle_zrot = [ chrono.ChQuaternionD() for i in range(len(leg_ang))]
      self.ankle_body = [chrono.ChBody() for i in range(len(leg_ang))]
      self.Ankle_rev = [chrono.ChLinkLockRevolute() for i in range(len(leg_ang))]
      self.ankle_motor = [chrono.ChLinkMotorRotationTorque() for i in range(len(leg_ang)) ]
      for i in range(len(leg_ang)):
             
             # Legs
             Leg_quat[i].Q_from_AngAxis(-leg_ang[i] , chrono.ChVectorD(0, 1, 0))
             self.leg_pos[i] = chrono.ChVectorD( (0.5*self.leg_length+self.abdomen_x)*math.cos(leg_ang[i]) ,self.abdomen_y0, (0.5*self.leg_length+self.abdomen_z)*math.sin(leg_ang[i]))
             self.leg_body[i].SetPos(self.leg_pos[i])
             self.leg_body[i].SetRot(Leg_quat[i])
             self.leg_body[i].AddAsset(self.leg_shape)
             self.leg_body[i].SetMass(self.leg_mass)
             self.leg_body[i].SetInertiaXX(self.leg_inertia)
             self.ant_sys.Add(self.leg_body[i])
             x_rel.append( Leg_quat[i].Rotate(chrono.ChVectorD(1, 0, 0)))
             z_rel.append( Leg_quat[i].Rotate(chrono.ChVectorD(0, 0, 1)))
             Leg_qa[i].Q_from_AngAxis(-leg_ang[i] , chrono.ChVectorD(0, 1, 0))
             z2x_leg[i].Q_from_AngAxis(chrono.CH_C_PI / 2 , x_rel[i])
             Leg_q[i] = z2x_leg[i] * Leg_qa[i] 
             Leg_rev_pos.append(chrono.ChVectorD(self.leg_pos[i]-chrono.ChVectorD(math.cos(leg_ang[i])*self.leg_length/2,0,math.sin(leg_ang[i])*self.leg_length/2)))
             Leg_chordsys.append(chrono.ChCoordsysD(Leg_rev_pos[i], Leg_q[i]))
             self.legjoint_frame.append(chrono.ChFrameD(Leg_chordsys[i]))
             self.Leg_rev[i].Initialize(self.body_abdomen, self.leg_body[i],Leg_chordsys[i])
             self.ant_sys.Add(self.Leg_rev[i])
             self.leg_motor[i].Initialize(self.body_abdomen, self.leg_body[i],self.legjoint_frame[i])
             self.ant_sys.Add(self.leg_motor[i])
             # Ankles
             q_ankle_zrot[i].Q_from_AngAxis(-self.ankle_angle , z_rel[i])
             anklejoint_chordsys.append(chrono.ChCoordsysD(self.leg_body[i].GetPos()+ self.leg_body[i].GetRot().Rotate(chrono.ChVectorD(self.leg_length/2, 0, 0)) , q_ankle_zrot[i] * self.leg_body[i].GetRot() ))
             self.anklejoint_frame.append(chrono.ChFrameD(anklejoint_chordsys[i]))
             self.ankle_body[i].SetPos(self.anklejoint_frame[i].GetPos() + self.anklejoint_frame[i].GetRot().Rotate(chrono.ChVectorD(self.ankle_length/2, 0, 0)))
             self.ankle_body[i].SetRot(  self.anklejoint_frame[i].GetRot() )
             self.ankle_body[i].AddAsset(self.ankle_shape)
             self.ankle_body[i].SetMass(self.ankle_mass)
             self.ankle_body[i].SetInertiaXX(self.ankle_inertia)
             self.ant_sys.Add(self.ankle_body[i])
             self.Ankle_rev[i].Initialize(self.leg_body[i], self.ankle_body[i], anklejoint_chordsys[i])
             self.ant_sys.Add(self.Ankle_rev[i])
             self.ankle_motor[i].Initialize(self.leg_body[i], self.ankle_body[i],self.anklejoint_frame[i])
             self.ant_sys.Add(self.ankle_motor[i])
             # Feet collisions
             self.ankle_body[i].SetMaterialSurface(self.ant_material)
             self.ankle_body[i].SetCollide(True)
             self.ankle_body[i].GetCollisionModel().ClearModel()
             self.ankle_body[i].GetCollisionModel().AddSphere(self.ankle_radius, chrono.ChVectorD(self.ankle_length/2, 0, 0 ) )
             self.ankle_body[i].GetCollisionModel().BuildModel()
             self.ankle_body[i].AddAsset(self.ankle_shape)
             
             self.ankle_body[i].AddAsset(self.foot_shape)
             
             self.Leg_rev[i].GetLimit_Rz().SetActive(True)
             self.Leg_rev[i].GetLimit_Rz().SetMin(-math.pi/3)
             self.Leg_rev[i].GetLimit_Rz().SetMax(math.pi/3)
             self.Ankle_rev[i].GetLimit_Rz().SetActive(True)
             self.Ankle_rev[i].GetLimit_Rz().SetMin(-math.pi/2)
             self.Ankle_rev[i].GetLimit_Rz().SetMax(math.pi/4)
             

           
    # Create the room floor: a simple fixed rigid body with a collision shape
    # and a visualization shape
      self.body_floor = chrono.ChBody()
      self.body_floor.SetBodyFixed(True)
      self.body_floor.SetPos(chrono.ChVectorD(0, -1, 0 ))
      self.body_floor.SetMaterialSurface(self.ant_material)
      
      # Floor Collision.
      self.body_floor.SetMaterialSurface(self.ant_material)
      self.body_floor.GetCollisionModel().ClearModel()
      self.body_floor.GetCollisionModel().AddBox(50, 1, 50, chrono.ChVectorD(0, 0, 0 ))
      self.body_floor.GetCollisionModel().BuildModel()
      self.body_floor.SetCollide(True)

    # Visualization shape
      body_floor_shape = chrono.ChBoxShape()
      body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(5, 1, 5)
      body_floor_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
      self.body_floor.GetAssets().push_back(body_floor_shape)
      body_floor_texture = chrono.ChTexture()
      body_floor_texture.SetTextureFilename('../data/grass.jpg')
      self.body_floor.GetAssets().push_back(body_floor_texture)     
      self.ant_sys.Add(self.body_floor)
      #self.body_abdomen.SetBodyFixed(True)
   
      if (self.animate):
            self.myapplication.AssetBindAll()
            self.myapplication.AssetUpdateAll()

      self.numsteps= 0
      self.step(np.zeros(8))
      return self.get_ob()

   def step(self, ac):
       xposbefore = self.body_abdomen.GetPos().x
       self.numsteps += 1
       if (self.animate):

              self.myapplication.GetDevice().run()
              self.myapplication.BeginScene()
              self.myapplication.DrawAll()
       self.ac = ac.reshape((-1,))
       for i in range(len(self.leg_motor)): 

              action_a = chrono.ChFunction_Const(self.gain*float(self.ac[i])) 
              action_b = chrono.ChFunction_Const(self.gain*float(self.ac[i+4])) 
              self.leg_motor[i].SetTorqueFunction(action_a)
              self.ankle_motor[i].SetTorqueFunction(action_b)


       if (self.animate):
              self.myapplication.DoStep()
              self.myapplication.EndScene()
       else:
              self.ant_sys.DoStepDynamics(self.timestep)

       obs= self.get_ob()
       rew = self.calc_rew(xposbefore)    
       
       self.is_done()
       return obs, rew, self.isdone, self.info
         
   def get_ob(self):
          

          ab_rot =  	self.body_abdomen.GetRot().Q_to_Euler123()
          ab_q = np.asarray([self.body_abdomen.GetPos().z, ab_rot.x, ab_rot.y, ab_rot.z])
          ab_speed = self.body_abdomen.GetRot().RotateBack(self.body_abdomen.GetPos_dt())
          ab_qdot = np.asarray([ ab_speed.x, ab_speed.y, ab_speed.z, self.body_abdomen.GetWvel_loc().x, self.body_abdomen.GetWvel_loc().y, self.body_abdomen.GetWvel_loc().z ])
          self.q_mot   = np.zeros([8,])
          self.q_dot_mot   = np.zeros([8,])
          joint_at_limit   = np.asarray([])
          feet_contact   = np.asarray([])
          for i in range(len(self.leg_motor)): 
                 
                 self.q_mot[i] = self.Leg_rev[i].GetRelAngle()
                 self.q_mot[i+4] = self.Ankle_rev[i].GetRelAngle() 
                 self.q_dot_mot[i]  = self.Leg_rev[i].GetRelWvel().z
                 self.q_dot_mot[i+4]  = self.Ankle_rev[i].GetRelWvel().z
                 joint_at_limit = np.append(joint_at_limit,  [ self.Leg_rev[i].GetLimit_Rz().GetMax()   < self.q_mot[i]   or self.Leg_rev[i].GetLimit_Rz().GetMin()   > self.q_mot[i] ,
                                                               self.Ankle_rev[i].GetLimit_Rz().GetMax() < self.q_mot[i+4] or self.Ankle_rev[i].GetLimit_Rz().GetMin() > self.q_mot[i+4]])
                 feet_contact = np.append(feet_contact, [self.ankle_body[i].GetContactForce().Length()] )
           

          feet_contact = np.clip(feet_contact , -5, 5)
          self.joint_at_limit = np.count_nonzero(np.abs(joint_at_limit))

          return np.concatenate ([ab_q, ab_qdot, self.q_mot,  self.q_dot_mot, feet_contact])
   
   def calc_rew(self, xposbefore):
                  
                  electricity_cost     = -2.0    # cost for using motors -- this parameter should be carefully tuned against reward for making progress, other values less improtant
                  #stall_torque_cost    = -0.1    # cost for running electric current through a motor even at zero rotational speed, small

                  joints_at_limit_cost = -0.2    # discourage stuck joints
                  
                  power_cost  = electricity_cost  * float(np.abs(self.ac*self.q_dot_mot).mean())  # let's assume we have DC motor with controller, and reverse current braking. BTW this is the formula of motor power
                  #Reduced stall cost to avoid joints at limit
                  joints_limit = joints_at_limit_cost * self.joint_at_limit
                  self.alive_bonus =  +1 if self.body_abdomen.GetContactForce().Length() == 0 else -1
                  progress = self.calc_progress()
                  rew = progress + self.alive_bonus + 0.1*(power_cost) + 3*(joints_limit)
                  return rew
   def calc_progress(self):
              d = np.linalg.norm( [self.Ytarg - self.body_abdomen.GetPos().y, self.Xtarg - self.body_abdomen.GetPos().x] )
              progress = -(d - self.d_old )/self.timestep
              self.d_old = d
              return progress                     
   def is_done(self):
          
          if ( self.alive_bonus < 0 or self.body_abdomen.GetPos().y > 49 or self.body_abdomen.GetPos().x > 49 or self.numsteps *self.timestep>100):
                 self.isdone = True

          
   def __del__(self):
          if (self.animate):
               self.myapplication.GetDevice().closeDevice()
               print('Destructor called, Device deleted.')
          else:
               print('Destructor called, No device to delete.')
        
   def ScreenCapture(self, interval):
          try: 
              self.myapplication.SetVideoframeSave(True)
              self.myapplication.SetVideoframeSaveInterval(interval)
          except:
                 print('No ChIrrApp found. Cannot save video frames.')
                     