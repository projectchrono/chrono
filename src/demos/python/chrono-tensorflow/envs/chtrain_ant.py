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
      self.ant_sys.GetSolver().AsIterative().SetMaxIterations(70)
      

      self.ant_material = chrono.ChContactMaterialNSC()
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

      self.abdomen_mass = self.abdomen_density * ((4/3)*chrono.CH_PI*self.abdomen_x*self.abdomen_y*self.abdomen_z)
      self.abdomen_inertia = chrono.ChVector3d((1/5)*self.abdomen_mass*(pow(self.abdomen_y,2)+pow(self.abdomen_z,2)),(1/5)*self.abdomen_mass*(pow(self.abdomen_x,2)+pow(self.abdomen_z,2)),(1/5)*self.abdomen_mass*(pow(self.abdomen_y,2)+pow(self.abdomen_x,2)))
      self.leg_mass = self.leg_density * self.leg_length * math.pi* pow (self.leg_radius,2)
      self.leg_inertia = chrono.ChVector3d(0.5*self.leg_mass*pow(self.leg_radius,2), (self.leg_mass/12)*(3*pow(self.leg_radius,2)+pow(self.leg_length,2)),(self.leg_mass/12)*(3*pow(self.leg_radius,2)+pow(self.leg_length,2)))
      self.ankle_mass = self.leg_density * self.ankle_length * math.pi* pow (self.ankle_radius,2)
      self.ankle_inertia = chrono.ChVector3d(0.5*self.ankle_mass*pow(self.ankle_radius,2), (self.ankle_mass/12)*(3*pow(self.ankle_radius,2)+pow(self.ankle_length,2)),(self.ankle_mass/12)*(3*pow(self.ankle_radius,2)+pow(self.ankle_length,2)))
      
      self.leg_limit = chrono.ChLinkLimit()
      self.ankle_limit = chrono.ChLinkLimit()
      self.leg_limit.SetDampingCoefficientMax(math.pi/9)
      self.leg_limit.SetDampingCoefficientMin(-math.pi/9)
      self.ankle_limit.SetDampingCoefficientMax(math.pi/9)
      self.ankle_limit.SetDampingCoefficientMin(-math.pi/9)
      
      if (self.animate) :
          self.vis = chronoirr.ChVisualSystemIrrlicht()
          self.vis.AttachSystem(self.ant_sys)
          self.vis.Initialize()
          self.vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
          self.vis.AddSkyBox()
          self.vis.AddCamera(chrono.ChVector3d(0,1.5,0))
          self.vis.AddTypicalLights()

   def reset(self):
    
      self.isdone = False
      self.ant_sys.Clear()
      self.body_abdomen = chrono.ChBody()
      self.body_abdomen.SetPos(chrono.ChVector3d(0, self.abdomen_y0, 0 ))
      self.body_abdomen.SetMass(self.abdomen_mass)
      self.body_abdomen.SetInertiaXX(self.abdomen_inertia)
    # set collision surface properties
      self.abdomen_shape = chrono.ChVisualShapeEllipsoid(self.abdomen_x, self.abdomen_y, self.abdomen_z)
      self.body_abdomen.AddVisualShape(self.abdomen_shape)
      self.body_abdomen.EnableCollision(True)
      body_abdomen_ct_shape = chrono.ChCollisionShapeEllipsoid(self.ant_material, self.abdomen_x, self.abdomen_y, self.abdomen_z)
      self.body_abdomen.AddCollisionShape(body_abdomen_ct_shape)
      self.ant_sys.Add(self.body_abdomen)
      
      
      leg_ang =  (1/4)*math.pi+(1/2)*math.pi*np.array([0,1,2,3])
      Leg_quat = [chrono.ChQuaterniond() for i in range(len(leg_ang))]
      self.leg_body = [chrono.ChBody() for i in range(len(leg_ang))]
      self.leg_pos= [chrono.ChVector3d() for i in range(len(leg_ang))]
      leg_cyl = chrono.ChCylinder(self.leg_radius, self.leg_length) 
      self.leg_shape = chrono.ChVisualShapeCylinder(leg_cyl)
      ankle_cyl = chrono.ChCylinder(self.ankle_radius, self.ankle_length) 
      self.ankle_shape = chrono.ChVisualShapeCylinder(ankle_cyl)
      foot_sphere = chrono.ChSphere(self.ankle_radius )
      self.foot_shape = chrono.ChVisualShapeSphere(foot_sphere)
      Leg_qa = [ chrono.ChQuaterniond()  for i in range(len(leg_ang))]
      Leg_q = [ chrono.ChQuaterniond()  for i in range(len(leg_ang))]
      z2x_leg = [ chrono.ChQuaterniond() for i in range(len(leg_ang))]
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
      q_ankle_zrot = [ chrono.ChQuaterniond() for i in range(len(leg_ang))]
      self.ankle_body = [chrono.ChBody() for i in range(len(leg_ang))]
      self.Ankle_rev = [chrono.ChLinkLockRevolute() for i in range(len(leg_ang))]
      self.ankle_motor = [chrono.ChLinkMotorRotationTorque() for i in range(len(leg_ang)) ]
      for i in range(len(leg_ang)):
             
             # Legs
             Leg_quat[i].SetFromAngleAxis(-leg_ang[i] , chrono.ChVector3d(0, 1, 0))
             self.leg_pos[i] = chrono.ChVector3d( (0.5*self.leg_length+self.abdomen_x)*math.cos(leg_ang[i]) ,self.abdomen_y0, (0.5*self.leg_length+self.abdomen_z)*math.sin(leg_ang[i]))
             self.leg_body[i].SetPos(self.leg_pos[i])
             self.leg_body[i].SetRot(Leg_quat[i])
             self.leg_body[i].AddVisualShape(self.leg_shape)
             self.leg_body[i].SetMass(self.leg_mass)
             self.leg_body[i].SetInertiaXX(self.leg_inertia)
             self.ant_sys.Add(self.leg_body[i])
             x_rel.append( Leg_quat[i].Rotate(chrono.ChVector3d(1, 0, 0)))
             z_rel.append( Leg_quat[i].Rotate(chrono.ChVector3d(0, 0, 1)))
             Leg_qa[i].SetFromAngleAxis(-leg_ang[i] , chrono.ChVector3d(0, 1, 0))
             z2x_leg[i].SetFromAngleAxis(chrono.CH_PI / 2 , x_rel[i])
             Leg_q[i] = z2x_leg[i] * Leg_qa[i] 
             Leg_rev_pos.append(chrono.ChVector3d(self.leg_pos[i]-chrono.ChVector3d(math.cos(leg_ang[i])*self.leg_length/2,0,math.sin(leg_ang[i])*self.leg_length/2)))
             Leg_chordsys.append(chrono.ChCoordsysd(Leg_rev_pos[i], Leg_q[i]))
             self.legjoint_frame.append(chrono.ChFramed(Leg_chordsys[i]))
             self.Leg_rev[i].Initialize(self.body_abdomen, self.leg_body[i],chrono.ChFramed(Leg_chordsys[i]))
             self.ant_sys.Add(self.Leg_rev[i])
             self.leg_motor[i].Initialize(self.body_abdomen, self.leg_body[i],self.legjoint_frame[i])
             self.ant_sys.Add(self.leg_motor[i])
             # Ankles
             q_ankle_zrot[i].SetFromAngleAxis(-self.ankle_angle , z_rel[i])
             anklejoint_chordsys.append(chrono.ChCoordsysd(self.leg_body[i].GetPos()+ self.leg_body[i].GetRot().Rotate(chrono.ChVector3d(self.leg_length/2, 0, 0)) , q_ankle_zrot[i] * self.leg_body[i].GetRot() ))
             self.anklejoint_frame.append(chrono.ChFramed(anklejoint_chordsys[i]))
             self.ankle_body[i].SetPos(self.anklejoint_frame[i].GetPos() + self.anklejoint_frame[i].GetRot().Rotate(chrono.ChVector3d(self.ankle_length/2, 0, 0)))
             self.ankle_body[i].SetRot(  self.anklejoint_frame[i].GetRot() )
             self.ankle_body[i].AddVisualShape(self.ankle_shape)
             self.ankle_body[i].SetMass(self.ankle_mass)
             self.ankle_body[i].SetInertiaXX(self.ankle_inertia)
             self.ant_sys.Add(self.ankle_body[i])
             self.Ankle_rev[i].Initialize(self.leg_body[i], self.ankle_body[i], chrono.ChFramed(anklejoint_chordsys[i]))
             self.ant_sys.Add(self.Ankle_rev[i])
             self.ankle_motor[i].Initialize(self.leg_body[i], self.ankle_body[i],self.anklejoint_frame[i])
             self.ant_sys.Add(self.ankle_motor[i])
             # Feet collisions
             self.ankle_body[i].EnableCollision(True)
             ankle_ct_shape = chrono.ChCollisionShapeSphere(self.ant_material, self.ankle_radius)
             self.ankle_body[i].AddCollisionShape(ankle_ct_shape, chrono.ChFramed(chrono.ChVector3d(self.ankle_length/2, 0, 0), chrono.QUNIT))
             self.ankle_body[i].AddVisualShape(self.ankle_shape, chrono.ChFramed(chrono.ChVector3d(self.ankle_length/2, 0, 0 )))
             
             self.ankle_body[i].AddVisualShape(self.foot_shape)
             
             self.Leg_rev[i].LimitRz().SetActive(True)
             self.Leg_rev[i].LimitRz().SetMin(-math.pi/3)
             self.Leg_rev[i].LimitRz().SetMax(math.pi/3)
             self.Ankle_rev[i].LimitRz().SetActive(True)
             self.Ankle_rev[i].LimitRz().SetMin(-math.pi/2)
             self.Ankle_rev[i].LimitRz().SetMax(math.pi/4)
             

           
    # Create the room floor: a simple fixed rigid body with a collision shape
    # and a visualization shape
      self.body_floor = chrono.ChBody()
      self.body_floor.SetFixed(True)
      self.body_floor.SetPos(chrono.ChVector3d(0, -1, 0 ))
      
      # Floor Collision.
      body_floor_ct_shape = chrono.ChCollisionShapeBox(self.ant_material, 50, 1, 50)
      self.body_floor.AddCollisionShape(body_floor_ct_shape)
      self.body_floor.EnableCollision(True)

    # Visualization shape
      body_floor_shape = chrono.ChVisualShapeBox(10, 2, 10)
      body_floor_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
      body_floor_shape.SetTexture(chrono.GetChronoDataFile('vehicle/terrain/textures/grass.jpg'))
      self.body_floor.AddVisualShape(body_floor_shape)   
      self.ant_sys.Add(self.body_floor)
      #self.body_abdomen.SetFixed(True)
   
      if (self.animate):
            self.vis.BindAll()

      self.numsteps= 0
      self.step(np.zeros(8))
      return self.get_ob()

   def step(self, ac):
       xposbefore = self.body_abdomen.GetPos().x
       self.numsteps += 1
       if (self.animate):

              self.vis.Run()
              self.vis.BeginScene()
              self.vis.Render()
       self.ac = ac.reshape((-1,))
       for i in range(len(self.leg_motor)): 

              action_a = chrono.ChFunctionConst(self.gain*float(self.ac[i])) 
              action_b = chrono.ChFunctionConst(self.gain*float(self.ac[i+4])) 
              self.leg_motor[i].SetTorqueFunction(action_a)
              self.ankle_motor[i].SetTorqueFunction(action_b)


       if (self.animate):
              self.vis.EndScene()
       self.ant_sys.DoStepDynamics(self.timestep)

       obs= self.get_ob()
       rew = self.calc_rew(xposbefore)    
       
       self.is_done()
       return obs, rew, self.isdone, self.info
         
   def get_ob(self):
          

          ab_rot =  	self.body_abdomen.GetRot().GetCardanAnglesXYZ()
          ab_q = np.asarray([self.body_abdomen.GetPos().z, ab_rot.x, ab_rot.y, ab_rot.z])
          ab_speed = self.body_abdomen.GetRot().RotateBack(self.body_abdomen.GetPosDt())
          ab_qdot = np.asarray([ ab_speed.x, ab_speed.y, ab_speed.z, self.body_abdomen.GetAngVelLocal().x, self.body_abdomen.GetAngVelLocal().y, self.body_abdomen.GetAngVelLocal().z ])
          self.q_mot   = np.zeros([8,])
          self.q_dot_mot   = np.zeros([8,])
          joint_at_limit   = np.asarray([])
          feet_contact   = np.asarray([])
          for i in range(len(self.leg_motor)): 
                 
                 self.q_mot[i] = self.Leg_rev[i].GetRelAngle()
                 self.q_mot[i+4] = self.Ankle_rev[i].GetRelAngle() 
                 self.q_dot_mot[i]  = self.Leg_rev[i].GetRelativeAngVel().z
                 self.q_dot_mot[i+4]  = self.Ankle_rev[i].GetRelativeAngVel().z
                 joint_at_limit = np.append(joint_at_limit,  [ self.Leg_rev[i].LimitRz().GetMax()   < self.q_mot[i]   or self.Leg_rev[i].LimitRz().GetMin()   > self.q_mot[i] ,
                                                               self.Ankle_rev[i].LimitRz().GetMax() < self.q_mot[i+4] or self.Ankle_rev[i].LimitRz().GetMin() > self.q_mot[i+4]])
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
               self.vis.GetDevice().closeDevice()
               print('Destructor called, Device deleted.')
          else:
               print('Destructor called, No device to delete.')
                     