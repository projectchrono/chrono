import pychrono as chrono
from pychrono import irrlicht as chronoirr
import numpy as np


class Model(object):
   def __init__(self, render):
      self.render = render

      self.observation_space= np.empty([4,1])
      self.action_space= np.empty([1,1])
      self.info =  {}
      self.timestep = 0.01
    # ---------------------------------------------------------------------
    #
    #  Create the simulation system and add items
    #
      
      self.rev_pend_sys = chrono.ChSystemNSC()

      chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.001)
      chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.001)

    #rev_pend_sys.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN) # precise, more slow
      self.rev_pend_sys.SetMaxItersSolverSpeed(70)



    # Create a contact material (surface property)to share between all objects.
      self.rod_material = chrono.ChMaterialSurfaceNSC()
      self.rod_material.SetFriction(0.5)
      self.rod_material.SetDampingF(0.2)
      self.rod_material.SetCompliance (0.0000001)
      self.rod_material.SetComplianceT(0.0000001)



    # Create the set of rods in a vertical stack, along Y axis


      self.size_rod_y = 2.0
      self.radius_rod = 0.05
      self.density_rod = 50;    # kg/m^3

      self.mass_rod = self.density_rod * self.size_rod_y *chrono.CH_C_PI* (self.radius_rod**2);  
      self.inertia_rod_y = (self.radius_rod**2) * self.mass_rod/2;
      self.inertia_rod_x = (self.mass_rod/12)*((self.size_rod_y**2)+3*(self.radius_rod**2))
      
      self.size_table_x = 0.3;
      self.size_table_y = 0.3;
      self.size_table_z = 0.3;

      if self.render:
             
             self.myapplication = chronoirr.ChIrrApp(self.rev_pend_sys)
             self.myapplication.AddShadowAll();
             self.myapplication.SetStepManage(True)
             self.myapplication.SetTimestep(0.01)
             self. myapplication.SetTryRealtime(True)
             
             self.myapplication.AddTypicalSky('../data/skybox/')
             self.myapplication.AddTypicalCamera(chronoirr.vector3df(0.5,0.5,1.0))
             self.myapplication.AddLightWithShadow(chronoirr.vector3df(2,4,2),    # point
                                            chronoirr.vector3df(0,0,0),    # aimpoint
                                            9,                 # radius (power)
                                            1,9,               # near, far
                                            30)                # angle of FOV

   def reset(self):
      #print("reset")
      self.isdone = False
      self.rev_pend_sys.Clear()
            # create it
      self.body_rod = chrono.ChBody()
    # set initial position
      self.body_rod.SetPos(chrono.ChVectorD(0, self.size_rod_y/2, 0 ))
    # set mass properties
      self.body_rod.SetMass(self.mass_rod)

      self.body_rod.SetInertiaXX(chrono.ChVectorD(self.inertia_rod_x,self.inertia_rod_y,self.inertia_rod_x))
    # set collision surface properties
      self.body_rod.SetMaterialSurface(self.rod_material)



    # Visualization shape, for rendering animation

      self.cyl_base1= chrono.ChVectorD(0, -self.size_rod_y/2, 0 )
      self.cyl_base2= chrono.ChVectorD(0, self.size_rod_y/2, 0 )

      self.body_rod_shape = chrono.ChCylinderShape()
      self.body_rod_shape.GetCylinderGeometry().p1= self.cyl_base1
      self.body_rod_shape.GetCylinderGeometry().p2= self.cyl_base2
      self.body_rod_shape.GetCylinderGeometry().rad= self.radius_rod

      self.body_rod.AddAsset(self.body_rod_shape)
      self.rev_pend_sys.Add(self.body_rod)


      self.body_floor = chrono.ChBody()
      self.body_floor.SetBodyFixed(True)
      self.body_floor.SetPos(chrono.ChVectorD(0, -5, 0 ))
      self.body_floor.SetMaterialSurface(self.rod_material)



      if self.render:
             self.body_floor_shape = chrono.ChBoxShape()
             self.body_floor_shape.GetBoxGeometry().Size = chrono.ChVectorD(3, 1, 3)
             self.body_floor.GetAssets().push_back(self.body_floor_shape)
             self.body_floor_texture = chrono.ChTexture()
             self.body_floor_texture.SetTextureFilename('../../../data/concrete.jpg')
             self.body_floor.GetAssets().push_back(self.body_floor_texture)

      self.rev_pend_sys.Add(self.body_floor)



      self.body_table = chrono.ChBody()
      self.body_table.SetPos(chrono.ChVectorD(0, -self.size_table_y/2, 0 ))
      self.body_table.SetMaterialSurface(self.rod_material)


      if self.render:
             self.body_table_shape = chrono.ChBoxShape()
             self.body_table_shape.GetBoxGeometry().Size = chrono.ChVectorD(self.size_table_x/2, self.size_table_y/2, self.size_table_z/2)
             self.body_table_shape.SetColor(chrono.ChColor(0.4,0.4,0.5))
             self.body_table.GetAssets().push_back(self.body_table_shape)
       
             self.body_table_texture = chrono.ChTexture()
             self.body_table_texture.SetTextureFilename('../../../data/concrete.jpg')
             self.body_table.GetAssets().push_back(self.body_table_texture)
      self.body_table.SetMass(0.1)
      self.rev_pend_sys.Add(self.body_table)



      self.link_slider = chrono.ChLinkLockPrismatic()
      z2x = chrono.ChQuaternionD()
      z2x.Q_from_AngAxis(-chrono.CH_C_PI / 2 , chrono.ChVectorD(0, 1, 0))

      self.link_slider.Initialize(self.body_table, self.body_floor, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), z2x))
      self.rev_pend_sys.Add(self.link_slider)


      self.act_initpos = chrono.ChVectorD(0,0,0)
      self.actuator = chrono.ChLinkMotorLinearForce()
      self.actuator.Initialize(self.body_table, self.body_floor, chrono.ChFrameD(self.act_initpos))
      self.rev_pend_sys.Add(self.actuator)

      self.rod_pin = chrono.ChMarker()
      self.body_rod.AddMarker(self.rod_pin)
      self.rod_pin.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0)))

      self.table_pin = chrono.ChMarker()
      self.body_table.AddMarker(self.table_pin)
      self.table_pin.Impose_Abs_Coord(chrono.ChCoordsysD(chrono.ChVectorD(0,0,0)))

      self.pin_joint = chrono.ChLinkLockRevolute()
      self.pin_joint.Initialize(self.rod_pin, self.table_pin)
      self.rev_pend_sys.Add(self.pin_joint)
      
      if self.render:

           # ---------------------------------------------------------------------
           #
           #  Create an Irrlicht application to visualize the system
           #
           # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
           # in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
           # If you need a finer control on which item really needs a visualization proxy
           # Irrlicht, just use application.AssetBind(myitem); on a per-item basis.
       
             self.myapplication.AssetBindAll();
       
                       # ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
                       # that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!
       
             self.myapplication.AssetUpdateAll();

	
      self.isdone= False
      self.steps= 0
      self.step(np.array([[0]]))
      return self.get_ob()

   def step(self, ac):
       
       action=float(ac[0])
       self.steps += 1
       self.ac = chrono.ChFunction_Const(action)
       self.actuator.SetForceFunction(self.ac)
       self.omega = self.pin_joint.GetRelWvel().Length()  
       
       if self.render:
              self.myapplication.GetDevice().run()
              self.myapplication.BeginScene()
              self.myapplication.DrawAll()
              self.myapplication.DoStep()
       else:
              self.rev_pend_sys.DoStepDynamics(self.timestep)
       self.rew = 1.0
                  
       self.obs= self.get_ob()
       if self.render:
              self.myapplication.EndScene()
       self.is_done()
       return self.obs, self.rew, self.isdone, self.info
       

              

   def get_ob(self):
           

          self.state = [self.link_slider.GetDist(), self.link_slider.GetDist_dt(), self.pin_joint.GetRelAngle(), self.omega]
          return np.asarray(self.state)

                 
   def is_done(self):
          if abs(self.link_slider.GetDist()) > 2 or self.steps> 100000 or abs(self.pin_joint.GetRelAngle()) >  0.2  :
                 self.isdone = True
   
   def ScreenCapture(self, interval):
          try: 
              self.myapplication.SetVideoframeSave(True)
              self.myapplication.SetVideoframeSaveInterval(interval)
              
          except:
                 print('No ChIrrApp found. Cannot save video frames.')
                     
       
   def __del__(self):
        if self.render:
            self.myapplication.GetDevice().closeDevice()
            print('Destructor called, Device deleted.')
        else:
            print('Destructor called, No device to delete.')
        