Make a spider robot in SolidWorks and simulate it {#tutorial_chrono_pyengine_demo_spider_robot}
==========================

This demo is about the simulation of a crawling spider robot with six legs,
where we control the motion of the legs with 18 actuators.

![](http://projectchrono.org/assets/manual/Tutorial_spider_robot.jpg)

In deail, one performs the following steps: 
- Use SolidWorks to make a 3D CAD model of the crawling robot, 
- export it as a .pyfile using the Chrono::SolidWorks add-in; ex. use the name **spider\_robot.py**
- create a Python program, ex. use the name **demo\_spider.py**, using the functions of [Chrono::PyEngine](@ref introduction_chrono_pyengine) to load and simulate spider\_robot.py.

For your convenience, the CAD model, the converted **spider\_robot.py** model, and the **demo\_spider.py** program are all available in the following archive:


[spider_robot.zip](http://projectchrono.org/assets/downloads/spider_robot.zip).


The following is the complete listing of **demo\_spider.py**  (it must stay in the same directory where you exported your CAD model with the name **spider\_robot.py**)

Note how we used  **mybody = mysystem.SearchBody('...')**; and  **mymarker = mybody.SearchMarker('...');** to retrieve object pointers from their names in the 3D CAD model. Also note that a part that shows as *M-410iB-300 -1/ArmBase<1>* in the GUI of SolidWorks, becomes *M-410iB-300 -1/ArmBase-1* for the Python side; i.e. the <N> suffix becomes -N.
	
Finally, note how we used ChFunction objects to build some basic loop motions for the actuators, in prescribed rotation mode.

~~~~~~~~~~~~~{.py}


def main():
    pass

if __name__ == '__main__':
    main()


import os
import math
import ChronoEngine_python_core as chrono
import ChronoEngine_python_postprocess as postprocess
import ChronoEngine_python_irrlicht as chronoirr

print ("Load a model exported by SolidWorks");



# ---------------------------------------------------------------------
#
#  Create the simulation system and add items
#

mysystem      = chrono.ChSystem()

parts = chrono.ImportSolidWorksSystem('./spider_robot');

for ib in parts:
    mysystem.Add(ib);

# Retrieve objects from their name as saved from the SolidWorks interface
# (look the spider_robot.py file to guess them, or look their name in SW)

bbody    = mysystem.SearchBody('Part3^SPIDER_ROBOT-1');
bbody.SetBodyFixed(False);
b1base    = mysystem.SearchBody('M-410iB-300 -1/ArmBase-1');
b1turret  = mysystem.SearchBody('M-410iB-300 -1/M-410iB-300-02-1');
b1bicept  = mysystem.SearchBody('M-410iB-300 -1/M-410iB-300-03-1');
b1forearm = mysystem.SearchBody('M-410iB-300 -1/M-410iB-300-06-1');
m1_1B = b1base.   SearchMarker('marker_M1_B');
m1_1A = b1turret. SearchMarker('marker_M1_A');
m1_2B = b1turret. SearchMarker('marker_M2_B');
m1_2A = b1bicept. SearchMarker('marker_M2_A');
m1_3B = b1bicept. SearchMarker('marker_M3_B');
m1_3A = b1forearm.SearchMarker('marker_M3_A');
b2base    = mysystem.SearchBody('M-410iB-300 -2/ArmBase-1');
b2turret  = mysystem.SearchBody('M-410iB-300 -2/M-410iB-300-02-1');
b2bicept  = mysystem.SearchBody('M-410iB-300 -2/M-410iB-300-03-1');
b2forearm = mysystem.SearchBody('M-410iB-300 -2/M-410iB-300-06-1');
m2_1B = b2base.   SearchMarker('marker_M1_B');
m2_1A = b2turret. SearchMarker('marker_M1_A');
m2_2B = b2turret. SearchMarker('marker_M2_B');
m2_2A = b2bicept. SearchMarker('marker_M2_A');
m2_3B = b2bicept. SearchMarker('marker_M3_B');
m2_3A = b2forearm.SearchMarker('marker_M3_A');
b3base    = mysystem.SearchBody('M-410iB-300 -3/ArmBase-1');
b3turret  = mysystem.SearchBody('M-410iB-300 -3/M-410iB-300-02-1');
b3bicept  = mysystem.SearchBody('M-410iB-300 -3/M-410iB-300-03-1');
b3forearm = mysystem.SearchBody('M-410iB-300 -3/M-410iB-300-06-1');
m3_1B = b3base.   SearchMarker('marker_M1_B');
m3_1A = b3turret. SearchMarker('marker_M1_A');
m3_2B = b3turret. SearchMarker('marker_M2_B');
m3_2A = b3bicept. SearchMarker('marker_M2_A');
m3_3B = b3bicept. SearchMarker('marker_M3_B');
m3_3A = b3forearm.SearchMarker('marker_M3_A');
b7base    = mysystem.SearchBody('M-410iB-300 -7/ArmBase-1');
b7turret  = mysystem.SearchBody('M-410iB-300 -7/M-410iB-300-02-1');
b7bicept  = mysystem.SearchBody('M-410iB-300 -7/M-410iB-300-03-1');
b7forearm = mysystem.SearchBody('M-410iB-300 -7/M-410iB-300-06-1');
m7_1B = b7base.   SearchMarker('marker_M1_B');
m7_1A = b7turret. SearchMarker('marker_M1_A');
m7_2B = b7turret. SearchMarker('marker_M2_B');
m7_2A = b7bicept. SearchMarker('marker_M2_A');
m7_3B = b7bicept. SearchMarker('marker_M3_B');
m7_3A = b7forearm.SearchMarker('marker_M3_A');
b8base    = mysystem.SearchBody('M-410iB-300 -8/ArmBase-1');
b8turret  = mysystem.SearchBody('M-410iB-300 -8/M-410iB-300-02-1');
b8bicept  = mysystem.SearchBody('M-410iB-300 -8/M-410iB-300-03-1');
b8forearm = mysystem.SearchBody('M-410iB-300 -8/M-410iB-300-06-1');
m8_1B = b8base.   SearchMarker('marker_M1_B');
m8_1A = b8turret. SearchMarker('marker_M1_A');
m8_2B = b8turret. SearchMarker('marker_M2_B');
m8_2A = b8bicept. SearchMarker('marker_M2_A');
m8_3B = b8bicept. SearchMarker('marker_M3_B');
m8_3A = b8forearm.SearchMarker('marker_M3_A');
b9base    = mysystem.SearchBody('M-410iB-300 -9/ArmBase-1');
b9turret  = mysystem.SearchBody('M-410iB-300 -9/M-410iB-300-02-1');
b9bicept  = mysystem.SearchBody('M-410iB-300 -9/M-410iB-300-03-1');
b9forearm = mysystem.SearchBody('M-410iB-300 -9/M-410iB-300-06-1');
m9_1B = b9base.   SearchMarker('marker_M1_B');
m9_1A = b9turret. SearchMarker('marker_M1_A');
m9_2B = b9turret. SearchMarker('marker_M2_B');
m9_2A = b9bicept. SearchMarker('marker_M2_A');
m9_3B = b9bicept. SearchMarker('marker_M3_B');
m9_3A = b9forearm.SearchMarker('marker_M3_A');



period = 2;
mfunc_sineS   = chrono.ChFunction_Sine(0, 1.0/period,  0.2);  # phase, frequency, amplitude
mfunc_swingSa = chrono.ChFunction_Repeat();
mfunc_swingSa.Set_fa(mfunc_sineS);
mfunc_swingSa.Set_window_length(period);
mfunc_swingSa.Set_window_start(0);
mfunc_swingSb = chrono.ChFunction_Repeat();
mfunc_swingSb.Set_fa(mfunc_sineS);
mfunc_swingSb.Set_window_length(period);
mfunc_swingSb.Set_window_start(period/2.0);
mfunc_sineD   = chrono.ChFunction_Sine(0, 1.0/period, -0.2);  # phase, frequency, amplitude
mfunc_swingDb = chrono.ChFunction_Repeat();
mfunc_swingDb.Set_fa(mfunc_sineD);
mfunc_swingDb.Set_window_length(period);
mfunc_swingDb.Set_window_start(period/2.0);
mfunc_swingDa = chrono.ChFunction_Repeat();
mfunc_swingDa.Set_fa(mfunc_sineD);
mfunc_swingDa.Set_window_length(period);
mfunc_swingDa.Set_window_start(0);


mfunc_sigma  = chrono.ChFunction_Sigma();
mfunc_sigma.Set_amp(-0.2);
mfunc_sigma.Set_end(0.5);
mfunc_const  = chrono.ChFunction_Const();
mfunc_sigmb  = chrono.ChFunction_Sigma();
mfunc_sigmb.Set_amp(0.2);
mfunc_sigmb.Set_end(0.5);
mfunc_seq    = chrono.ChFunction_Sequence();
mfunc_seq.InsertFunct(mfunc_sigma, 0.5, 1, True); # fx, duration, weight, C0 continuity
mfunc_seq.InsertFunct(mfunc_const, 1.0, 1, True); # fx, duration, weight, C0 continuity
mfunc_seq.InsertFunct(mfunc_sigmb, 0.5, 1, True); # fx, duration, weight, C0 continuity
mfunc_updownA = chrono.ChFunction_Repeat();
mfunc_updownA.Set_fa(mfunc_seq);
mfunc_updownA.Set_window_length(period);
mfunc_updownB = chrono.ChFunction_Repeat();
mfunc_updownB.Set_fa(mfunc_seq);
mfunc_updownB.Set_window_length(period);
mfunc_updownB.Set_window_phase(period/2.0);

# Add actuators to Leg n.1

motor1_1 = chrono.ChLinkEngine();
motor1_1.Initialize(m1_1A, m1_1B);
motor1_1.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor1_1.Set_rot_funct(mfunc_swingSa);
mysystem.Add(motor1_1);

motor1_2 = chrono.ChLinkEngine();
motor1_2.Initialize(m1_2A, m1_2B);
motor1_2.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor1_2.Set_rot_funct(mfunc_updownA);
mysystem.Add(motor1_2);

motor1_3 = chrono.ChLinkEngine();
motor1_3.Initialize(m1_3A, m1_3B);
motor1_3.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor1_3.Set_rot_funct(mfunc_const);
mysystem.Add(motor1_3);

# Add actuators to Leg n.2

motor2_1 = chrono.ChLinkEngine();
motor2_1.Initialize(m2_1A, m2_1B);
motor2_1.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor2_1.Set_rot_funct(mfunc_swingSb);
mysystem.Add(motor2_1);

motor2_2 = chrono.ChLinkEngine();
motor2_2.Initialize(m2_2A, m2_2B);
motor2_2.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor2_2.Set_rot_funct(mfunc_updownB);
mysystem.Add(motor2_2);

motor2_3 = chrono.ChLinkEngine();
motor2_3.Initialize(m2_3A, m2_3B);
motor2_3.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor2_3.Set_rot_funct(mfunc_const);
mysystem.Add(motor2_3);

# Add actuators to Leg n.3

motor3_1 = chrono.ChLinkEngine();
motor3_1.Initialize(m3_1A, m3_1B);
motor3_1.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor3_1.Set_rot_funct(mfunc_swingSa);
mysystem.Add(motor3_1);

motor3_2 = chrono.ChLinkEngine();
motor3_2.Initialize(m3_2A, m3_2B);
motor3_2.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor3_2.Set_rot_funct(mfunc_updownA);
mysystem.Add(motor3_2);

motor3_3 = chrono.ChLinkEngine();
motor3_3.Initialize(m3_3A, m3_3B);
motor3_3.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor3_3.Set_rot_funct(mfunc_const);
mysystem.Add(motor3_3);

# Add actuators to Leg n.9

motor9_1 = chrono.ChLinkEngine();
motor9_1.Initialize(m9_1A, m9_1B);
motor9_1.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor9_1.Set_rot_funct(mfunc_swingDb);
mysystem.Add(motor9_1);

motor9_2 = chrono.ChLinkEngine();
motor9_2.Initialize(m9_2A, m9_2B);
motor9_2.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor9_2.Set_rot_funct(mfunc_updownB);
mysystem.Add(motor9_2);

motor9_3 = chrono.ChLinkEngine();
motor9_3.Initialize(m9_3A, m9_3B);
motor9_3.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor9_3.Set_rot_funct(mfunc_const);
mysystem.Add(motor9_3);

# Add actuators to Leg n.8

motor8_1 = chrono.ChLinkEngine();
motor8_1.Initialize(m8_1A, m8_1B);
motor8_1.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor8_1.Set_rot_funct(mfunc_swingDa);
mysystem.Add(motor8_1);

motor8_2 = chrono.ChLinkEngine();
motor8_2.Initialize(m8_2A, m8_2B);
motor8_2.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor8_2.Set_rot_funct(mfunc_updownA);
mysystem.Add(motor8_2);

motor8_3 = chrono.ChLinkEngine();
motor8_3.Initialize(m8_3A, m8_3B);
motor8_3.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor8_3.Set_rot_funct(mfunc_const);
mysystem.Add(motor8_3);

# Add actuators to Leg n.7

motor7_1 = chrono.ChLinkEngine();
motor7_1.Initialize(m7_1A, m7_1B);
motor7_1.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor7_1.Set_rot_funct(mfunc_swingDb);
mysystem.Add(motor7_1);

motor7_2 = chrono.ChLinkEngine();
motor7_2.Initialize(m7_2A, m7_2B);
motor7_2.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor7_2.Set_rot_funct(mfunc_updownB);
mysystem.Add(motor7_2);

motor7_3 = chrono.ChLinkEngine();
motor7_3.Initialize(m7_3A, m7_3B);
motor7_3.Set_eng_mode(chrono.ChLinkEngine.ENG_MODE_ROTATION);
motor7_3.Set_rot_funct(mfunc_const);
mysystem.Add(motor7_3);

#
# Create a floor

mfloor = chrono.ChBody();
mfloor.SetBodyFixed(True);
mfloor.GetCollisionModel().ClearModel();
mfloor.GetCollisionModel().AddBox(10,0.5,10, chrono.ChVectorD(0,0.3,0));
mfloor.GetCollisionModel().BuildModel();
mfloor.SetCollide(True);
mysystem.Add(mfloor);

mfloorshape = chrono.ChBoxShape();
mfloorshape.GetBoxGeometry().Size = chrono.ChVectorD(10,0.5,10);
mfloorshape.GetBoxGeometry().Pos  = chrono.ChVectorD(0,0.3,0);
mfloor.AddAsset(mfloorshape);

mfloorcolor = chrono.ChColorAsset(0.2,0.2,0.2);
mfloor.AddAsset(mfloorcolor);

# ---------------------------------------------------------------------
#
#  Create an Irrlicht application to visualize the system
#

myapplication = chronoirr.ChIrrApp(mysystem, 'Test', chronoirr.dimension2du(1280,720))

myapplication.AddTypicalSky('./data/skybox/')
myapplication.AddTypicalCamera(chronoirr.vector3df(2.8,2.6,2.8),chronoirr.vector3df(0.0,2.6,0.0))
myapplication.AddTypicalLights()
myapplication.AddLightWithShadow(chronoirr.vector3df(10,20,10),chronoirr.vector3df(0,2.6,0), 10 ,10,40, 60, 512);

            # ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			# in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			# If you need a finer control on which item really needs a visualization proxy in
			# Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

myapplication.AssetBindAll();

			# ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			# that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

myapplication.AssetUpdateAll();

			# ==IMPORTANT!== Use this function for enabling cast soft shadows

myapplication.AddShadowAll();

# ---------------------------------------------------------------------
#
#  Run the simulation
#

mysystem.SetMaxItersSolverSpeed(600);
mysystem.SetSolverWarmStarting(True);
mysystem.SetSolverType(chrono.ChSolver.Type_BARZILAIBORWEIN);
myapplication.SetTimestep(0.002):


while(myapplication.GetDevice().run()):
    myapplication.BeginScene()
    myapplication.DrawAll()
    myapplication.DoStep()
    myapplication.EndScene()

~~~~~~~~~~~~~
