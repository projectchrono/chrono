# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Simone Benatti
# =============================================================================
#
#   Demo code about using motors to impose rotation or traslation between parts
#
# =============================================================================

import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import math as m
from copy import deepcopy

# Shortcut function that creates two bodies (a slider and a guide) in a given position,
# just to simplify the creation of multiple linear motors in this demo.
# (skip this and go to main() for the tutorial)
""" Passing the ChBody as argument and assigning a derivate class won't work in Python, and the reference to  the old body will go lost. 
Therefore we don't pass the bodies, but instantiate them in the function and pass them"""
def CreateSliderGuide(material,
                      msystem,
                      mpos) :
    mguide = chrono.ChBodyEasyBox(4, 0.3, 0.6, 1000, True, True, material)
    mguide.SetPos(mpos)
    mguide.SetBodyFixed(True)
    msystem.Add(mguide)

    mslider = chrono.ChBodyEasyBox(0.4, 0.2, 0.5, 1000, True, True, material)
    mslider.SetPos(mpos + chrono.ChVectorD(0, 0.3, 0))
    msystem.Add(mslider)

    mcolor = chrono.ChColorAsset(0.6, 0.6, 0.0)
    mslider.AddAsset(mcolor)

    obstacle = chrono.ChBodyEasyBox(0.4, 0.4, 0.4, 8000, True, True, material)
    obstacle.SetPos(mpos + chrono.ChVectorD(1.5, 0.4, 0))
    msystem.Add(obstacle)
    mcolorobstacle = chrono.ChColorAsset(0.2, 0.2, 0.2)
    mslider.AddAsset(mcolorobstacle)
    return mguide, mslider
    


# Shortcut function that creates two bodies (a stator and a rotor) in a given position,
# just to simplify the creation of multiple linear motors in this demo
# (skip this and go to main() for the tutorial)

def CreateStatorRotor(material,
                      msystem,
                      mpos) :
    mstator = chrono.ChBodyEasyCylinder(0.5, 0.1, 1000, True, True, material)
    mstator.SetPos(mpos)
    mstator.SetRot(chrono.Q_from_AngAxis(chrono.CH_C_PI_2, chrono.VECT_X))
    mstator.SetBodyFixed(True)
    msystem.Add(mstator)
    

    mrotor = chrono.ChBodyEasyBox(1, 0.1, 0.1, 1000, True, True, material)
    mrotor.SetPos(mpos + chrono.ChVectorD(0.5, 0, -0.15))
    msystem.Add(mrotor)

    mcolor = chrono.ChColorAsset(0.6, 0.6, 0.0)
    mrotor.AddAsset(mcolor)
    return mstator, mrotor

print("Copyright (c) 2017 projectchrono.org")

# Create a ChronoENGINE physical system
mphysicalSystem = chrono.ChSystemNSC()

# Contact material shared among all objects
material = chrono.ChMaterialSurfaceNSC()

# Create a floor that is fixed (that is used also to represent the absolute reference)
floorBody = chrono.ChBodyEasyBox(20, 2, 20, 3000, True, True, material)
floorBody.SetPos(chrono.ChVectorD(0, -2, 0))
floorBody.SetBodyFixed(True)
mphysicalSystem.Add(floorBody)

mtexture = chrono.ChTexture()
mtexture.SetTextureFilename(chrono.GetChronoDataFile("textures/blue.png"))
floorBody.AddAsset(mtexture)

# In the following we will create different types of motors
# - rotational motors: examples A.1, A.2, etc.
# - linear motors, examples B.1, B.2 etc.

# EXAMPLE A.1
#
# - class:   ChLinkMotorRotationSpeed
# - type:    rotational motor
# - control: impose a time-dependent speed=v(t)
#
# This is a simple type of rotational actuator. It assumes that
# you know the exact angular speed of the rotor respect to the stator,
# as a function of time:   angular speed = w(t).
# Use this to simulate fans, rotating cranks, etc.
# Note: this is a rheonomic motor that enforces the motion
# geometrically no compliance is allowed, this means that if the
# rotating body hits some hard contact, the solver might give unpredictable
# oscillatory or diverging results because of the contradiction.

positionA1 = chrono.ChVectorD(-3, 2, -3) 
stator1, rotor1 = CreateStatorRotor(material, mphysicalSystem, positionA1)

# Create the motor
rotmotor1 = chrono.ChLinkMotorRotationSpeed()

# Connect the rotor and the stator and add the motor to the system:
rotmotor1.Initialize(rotor1,                # body A (slave)
                      stator1,               # body B (master)
                      chrono.ChFrameD(positionA1)  # motor frame, in abs. coords
)
mphysicalSystem.Add(rotmotor1)

# Create a ChFunction to be used for the ChLinkMotorRotationSpeed
mwspeed = chrono.ChFunction_Const(chrono.CH_C_PI_2)  # constant angular speed, in [rad/s], 1PI/s =180°/s
# Let the motor use this motion function:
rotmotor1.SetSpeedFunction(mwspeed)

# The ChLinkMotorRotationSpeed contains a hidden state that performs the time integration
# of the angular speed setpoint: such angle is then imposed to the
# constraat the positional level too, thus avoiding angle error
# accumulation (angle drift). Optionally, such positional constraint
# level can be disabled as follows:
#
# rotmotor1.SetAvoidAngleDrift(False)

# EXAMPLE A.2
#
# - class:   ChLinkMotorRotationAngle
# - type:    rotational motor
# - control: impose a time-dependent angle=a(t)
#
# This is a simple type of rotational actuator. It assumes that
# you know the exact angular angle of the rotor respect to the stator,
# as a function of time:   angle = a(t).
# Use this to simulate servo drives in robotic systems and automation,
# where you can assume that the motor rotates with an infinitely stiff
# and reactive control, that exactly follows your prescribed motion profiles.
# Note: this is a rheonomic motor that enforces the motion
# geometrically no compliance is allowed, this means that if the
# rotating body hits some hard contact, the solver might give unpredictable
# oscillatory or diverging results because of the contradiction.

positionA2 = chrono.ChVectorD(-3, 2, -2)
stator2, rotor2 = CreateStatorRotor(material, mphysicalSystem, positionA2)

# Create the motor
rotmotor2 = chrono.ChLinkMotorRotationAngle()

# Connect the rotor and the stator and add the motor to the system:
rotmotor2.Initialize(rotor2,                # body A (slave)
                      stator2,               # body B (master)
                      chrono.ChFrameD(positionA2)  # motor frame, in abs. coords
)
mphysicalSystem.Add(rotmotor2)

# Create a ChFunction to be used for the ChLinkMotorRotationAngle
msineangle = chrono.ChFunction_Sine(0,       # phase [rad]
                                    0.05,    # frequency [Hz]
                                    chrono.CH_C_PI)  # amplitude [rad]

# Let the motor use this motion function as a motion profile:
rotmotor2.SetAngleFunction(msineangle)

# EXAMPLE A.3
#
# - class:   ChLinkMotorRotationTorque
# - type:    rotational motor
# - control: impose a (time-dependent) torque=T(t)
#
# For this motor, you must specify a time-dependent torque as torque = T(t).
# (If you want to use this motor to follow some desired motion profiles, you
# must implement a PID controller that continuously adjusts the value of the
# torque during the simulation).

positionA3 = chrono.ChVectorD(-3, 2, -1)
stator3, rotor3 = CreateStatorRotor(material, mphysicalSystem, positionA3)

# Create the motor
rotmotor3 = chrono.ChLinkMotorRotationTorque()

# Connect the rotor and the stator and add the motor to the system:
rotmotor3.Initialize(rotor3,                # body A (slave)
                      stator3,               # body B (master)
                      chrono.ChFrameD(positionA3))  # motor frame, in abs. coords

mphysicalSystem.Add(rotmotor3)

# The torque(time) function:
mtorquetime = chrono.ChFunction_Sine(0,   # phase [rad]
                                     2,   # frequency [Hz]
                                     160)  # amplitude [Nm]

# Let the motor use this motion function as a motion profile:
rotmotor3.SetTorqueFunction(mtorquetime)

# EXAMPLE A.4
#
# As before, use a ChLinkMotorRotationTorque, but this time compute
# torque by a custom function. In this example we implement a
# basic torque(speed) model of a three-phase induction electric motor..

positionA4 = chrono.ChVectorD(-3, 2, 0)
stator4, rotor4 =  CreateStatorRotor( material, mphysicalSystem, positionA4)

# Create the motor
rotmotor4 = chrono.ChLinkMotorRotationTorque()

# Connect the rotor and the stator and add the motor to the system:
rotmotor4.Initialize(rotor4,                # body A (slave)
                      stator4,               # body B (master)
                      chrono.ChFrameD(positionA4))  # motor frame, in abs. coords

mphysicalSystem.Add(rotmotor4)

# Implement our custom  torque function.
# We could use pre-defined ChFunction classes like sine, constant, ramp, etc.,
# but in this example we show how to implement a custom function: a
# torque(speed) function that represents a three-phase electric induction motor.
# Just inherit from ChFunction and implement Get_y() so that it returns different
# values (regrdless of time x) depending only on the slip speed of the motor:
class MyTorqueCurve(chrono.ChFunction) :
  def __init__(self, e2, r2, x2, n_s, mot) : 
        super().__init__()
        self.E2 = e2 
        self.R2 = r2
        self.X2 = x2
        self.ns = n_s
        self.mymotor = mot

  def Clone(self) :
        return deepcopy(self)

  def Get_y(self, x) :
        # The three-phase torque(speed) model
        w = self.mymotor.GetMotorRot_dt()
        s = (self.ns - w) / self.ns  # slip
        T = (3.0 / 2 * chrono.CH_C_PI * self.ns) * (s * self.E2 * self.E2 * self.R2) / (self.R2 * self.R2 + pow(s * self.X2, 2))  # electric torque curve
        T -= w * 5  # simulate also a viscous brake
        return T
    

# Create the function object from our custom class
mtorquespeed = MyTorqueCurve(120, 80, 1, 6, rotmotor4)


# Let the motor use this motion function as a motion profile:
rotmotor4.SetTorqueFunction(mtorquespeed)

# EXAMPLE A.5
#
#
# - class:   ChLinkMotorRotationDriveline
# - type:    rotational motor
# - control: delegated to an embedded user-defined driveline/powertrain
#
# This is the most powerful motor type. It allows the creation of
# generic 1D powertrain inside this 3D motor.
# Powertrains/drivelines are defined by connecting a variable number of
# 1D objects such as ChShaft, ChClutch, ChShaftsMotor, etc. In this way, for
# example, you can represent a drive+flywheel+reducer, hence taking into account
# of the inertia of the flywheel without the complication of adding a full 3D shape that
# represents the flywheel, and withoput needing 3D constrafor gears, bearings, etc.
# The 1D driveline is "interfaced" to the two connected threedimensional
# parts using two "inner" 1D shafts, each rotating as the connected 3D part
# it is up to the user to build the driveline that connects those two shafts.
# Most often the driveline is like a graph starting at inner shaft 2 (consider
# it to be the truss for holding the motor drive, also the support for reducers
# if any) and ending at inner shaft 1 (consider it to be the output, i.e. the
# slow-rotation spindle).

positionA5 = chrono.ChVectorD(-3, 2, 1)
stator5, rotor5 = CreateStatorRotor(material, mphysicalSystem, positionA5)

# Create the motor
rotmotor5 = chrono.ChLinkMotorRotationDriveline()

# Connect the rotor and the stator and add the motor to the system:
rotmotor5.Initialize(rotor5,                # body A (slave)
                      stator5,               # body B (master)
                      chrono.ChFrameD(positionA5) ) # motor frame, in abs. coords

mphysicalSystem.Add(rotmotor5)

# You may want to change the inertia of 'inner' 1d shafts, (each has default 1kg/m^2)
# Note: they adds up to 3D inertia when 3D parts rotate about the link shaft.
# Note: do not use too small values compared to 3D inertias: it might negatively affect
# the precision of some solvers if so, rather diminish the 3D inertia of stator/rotor parts and add to these.
rotmotor5.GetInnerShaft1().SetInertia(0.2)  # [kg/m^2]
rotmotor5.GetInnerShaft2().SetInertia(0.2)  # [kg/m^2]

# Now create the driveline. We want to model a drive+reducer sytem.
# This driveline must connect "inner shafts" of s1 and s2, where:
#  s1, is the 3D "rotor5"  part A (ex. a robot arm) and
#  s2, is the 3D "stator5" part B (ex. a robot base).
# In the following scheme, the motor is [ DRIVE ], the reducer is [ REDUCER ],
# the shafts ( shown with symbol || to mean inertia) are:
#  S1: the 1D inner shaft for s1 robot arm (already present in ChLinkMotorRotationDriveline)
#  S2: the 1D inner shaft for s2 robot base (already present in ChLinkMotorRotationDriveline)
#  A : the shaft of the electric drive
#
#      S2                A                    S1
# 3d<--||---[ DRIVE ]---||-----[ REDUCER ]----||-.3d
# s2   ||                      [         ]         s1
#      ||----------------------[         ]
#

# Create 'A', a 1D shaft. This is the shaft of the electric drive, representing its inertia.

my_shaftA = chrono.ChShaft()
my_shaftA.SetInertia(0.03)
mphysicalSystem.Add(my_shaftA)

# Create 'DRIVE', the hi-speed motor model - as a simple example use a 'imposed speed' motor: this
# is the equivalent of the ChLinkMotorRotationSpeed, but for 1D elements:

my_drive = chrono.ChShaftsMotorSpeed()
my_drive.Initialize(my_shaftA,                   # A , the rotor of the drive
                     rotmotor5.GetInnerShaft2() ) # S2, the stator of the drive

mphysicalSystem.Add(my_drive)
# Create a speed(time) function, and use it in my_drive:
my_driveangle = chrono.ChFunction_Const(25 * chrono.CH_C_2PI)  # 25 [rps] = 1500 [rpm]
my_drive.SetSpeedFunction(my_driveangle)

# Create the REDUCER. We should not use the simple ChShaftsGear because
# it does not transmit torque to the support. So use ChShaftsPlanetary
# and use it in ordinary mode, keeping the carrier as truss: so it
# will connect three parts: the carrier(here the truss), the in shaft, the out shaft.

my_reducer = chrono.ChShaftsPlanetary()
my_reducer.Initialize(rotmotor5.GetInnerShaft2(),  # S2, the carrier (truss)
                       my_shaftA,                    # A , the input shaft
                       rotmotor5.GetInnerShaft1() )  # S1, the output shaft

my_reducer.SetTransmissionRatioOrdinary(1.0 / 100.0)  # ratio between wR/wA
mphysicalSystem.Add(my_reducer)

# Btw:  later, if you want, you can access / plot speeds and
# torques for whatever part of the driveline by putting lines like the following
# in the  while() {...} simulation loop:
#
# GetLog() << " 1D shaft 'A' angular speed: "      << my_shaftA.GetPos_dt() << " [rad/s] \n"
# GetLog() << " 1D Drive angular speed: rot-stat " << my_drive.GetMotorRot_dt() << " [rad/s] \n"
# GetLog() << " 1D Drive torque: "                 << my_drive.GetMotorTorque() << " [Ns] \n"
# GetLog() << " 3D motor angular speed: rot-stat " << rotmotor5.GetMotorRot_dt() << " [rad/s] \n"
# GetLog() << " 3D motor torque: "                 << rotmotor5.GetMotorTorque() << " [Ns] \n"
# etc.

# EXAMPLE B.1
#
# - class:   ChLinkMotorLinearPosition
# - type:    linear motor
# - control: impose a time-dependent position=f(t)
#
# This is the simpliest type of linear actuator. It assumes that
# you know the exact position of the slider respect to the guide,
# as a function of time:   position = f(t)
# Therefore, this is a rheonomic motor that enforces the motion
# geometrically no compliance is allowed, this means that if the
# sliding body hits some hard contact, the solver might give unpredictable
# oscillatory or diverging results because of the contradiction.

positionB1 = chrono.ChVectorD(0, 0, -3)
guide1, slider1 = CreateSliderGuide(material, mphysicalSystem, positionB1)

# Create the linear motor
motor1 = chrono.ChLinkMotorLinearPosition()

# Connect the guide and the slider and add the motor to the system:
motor1.Initialize(slider1,               # body A (slave)
                   guide1,                # body B (master)
                   chrono.ChFrameD(positionB1)  # motor frame, in abs. coords
)
mphysicalSystem.Add(motor1)

# Create a ChFunction to be used for the ChLinkMotorLinearPosition
msine = chrono.ChFunction_Sine(0,    # phase
                               0.5,  # frequency
                               1.6)   # amplitude

# Let the motor use this motion function:
motor1.SetMotionFunction(msine)

# EXAMPLE B.2
#
# - class:   ChLinkMotorLinearSpeed
# - type:    linear motor
# - control: impose a time-dependent speed=v(t)
#
# This is a simple type of linear actuator. It assumes that
# you know the exact speed of the slider respect to the guide,
# as a function of time:   speed = v(t)
# Therefore, this is a rheonomic motor that enforces the motion
# geometrically no compliance is allowed, this means that if the
# sliding body hits some hard contact, the solver might give unpredictable
# oscillatory or diverging results because of the contradiction.
# It contains a hidden state that performs the time integration
# of the required speed, such position is then imposed too to the
# constraat the positional level, thus avoiding position error
# accumulation (position drift). Optionally, such constraon
# position level can be disabled if you are not interested in pos.drift.

positionB2 = chrono.ChVectorD(0, 0, -2)
guide2, slider2 = CreateSliderGuide(material, mphysicalSystem, positionB2)

# Create the linear motor
motor2 = chrono.ChLinkMotorLinearSpeed()

# Connect the guide and the slider and add the motor to the system:
motor2.Initialize(slider2,               # body A (slave)
                   guide2,                # body B (master)
                   chrono.ChFrameD(positionB2) ) # motor frame, in abs. coords

mphysicalSystem.Add(motor2)

# Create a ChFunction to be used for the ChLinkMotorLinearSpeed
msp = chrono.ChFunction_Sine(chrono.CH_C_PI_2,            # phase
                             0.5,                  # frequency
                             1.6 * 0.5 * chrono.CH_C_2PI)  # amplitude

# Let the motor use this motion function:
motor2.SetSpeedFunction(msp)

# The ChLinkMotorLinearSpeed contains a hidden state that performs the time integration
# of the speed setpoint: such position is then imposed to the
# constraat the positional level too, thus avoiding position error
# accumulation (position drift). Optionally, such position constraint
# level can be disabled as follows:
#
# motor2.SetAvoidPositionDrift(False)

# EXAMPLE B.3
#
# - class:   ChLinkMotorLinearForce
# - type:    linear motor
# - control: impose a time-dependent force=F(t)
#
# This actuator is moved via force as a function of time, F=F(t).
# The basic "open loop" option is to provide a F(t) at the beginning (ex using
# a feedforward model), but then there is no guarantee about the
# precise position of the slider, when the simulation runs.
# This means that, unless you update the force F at each time
# step using some type of feedback controller, this actuator
# cannot be used to follow some position setpoint. Implementing
# your controller might complicate things, but it could be closer to
# the behavior of a real actuator, that have some delay, bandwidth
# latency and compliance - for example, differently from
# other types such as ChLinkMotorLinearPosition  and
# ChLinkMotorLinearSpeed, this force motor does not enforce any
# constraon the direction of motion, so if it the slider hits
# some hard contact, it just stops and keeps pushing, and no troubles
# with the solver happen.

positionB3 = chrono.ChVectorD(0, 0, -1)
guide3, slider3 = CreateSliderGuide(material, mphysicalSystem, positionB3)

# just for fun: modify the initial speed of slider to match other examples
slider3.SetPos_dt(chrono.ChVectorD(1.6 * 0.5 * chrono.CH_C_2PI))

# Create the linear motor
motor3 = chrono.ChLinkMotorLinearForce()

# Connect the guide and the slider and add the motor to the system:
motor3.Initialize(slider3,               # body A (slave)
                   guide3,                # body B (master)
                   chrono.ChFrameD(positionB3) ) # motor frame, in abs. coords

mphysicalSystem.Add(motor3)

# Create a ChFunction to be used for F(t) in ChLinkMotorLinearForce.
mF = chrono.ChFunction_Const(200)
# Let the motor use this motion function:
motor3.SetForceFunction(mF)

# Alternative: just for fun, use a sine harmonic whose max force is F=M*A, where
# M is the mass of the slider, A is the max acceleration of the previous examples,
# so finally the motion should be quite the same - but without feedback, if hits a disturb, it goes crazy:
mF2 =chrono.ChFunction_Sine(0,                                                        # phase
                            0.5,                                                      # frequency
                            slider3.GetMass() * 1.6 * pow(0.5 * chrono.CH_C_2PI, 2) ) # amplitude

# motor3.SetForceFunction(mF2) # uncomment to test this

# EXAMPLE B.4
#
# As before, use a ChLinkMotorLinearForce, but this time compute
# F by a user-defined procedure (as a callback). For example, here we write a very
# basic PID control algorithm that adjusts F trying to chase a sinusoidal position.

positionB4 = chrono.ChVectorD(0, 0, 0)
guide4, slider4 = CreateSliderGuide(material, mphysicalSystem, positionB4)

# Create the linear motor
motor4 = chrono.ChLinkMotorLinearForce()

# Connect the guide and the slider and add the motor to the system:
motor4.Initialize(slider4,                       # body A (slave)
                  guide4,                       # body B (master)
                  chrono.ChFrameD(positionB4))  # motor frame, in abs. coords

mphysicalSystem.Add(motor4)

# Create a ChFunction that computes F by a user-defined algorithm, as a callback.
# One quick option would be to inherit from the ChFunction base class, and implement the Get_y()
# function by putting the code you wish, as explained in demo_CH_functions.cpp. However this has some
# limitations. A more powerful approach is to inherit from ChFunction_SetpointCallback, that automatically
# computes the derivatives, if needed, by BDF etc. Therefore:
# 1. You must inherit from the ChFunction_SetpointCallback base class, and implement the SetpointCallback()
#    function by putting the code you wish. For example something like the follow:

class MyForceClass (chrono.ChFunction_SetpointCallback) :
      def __init__(self, amp, freq, prop, der, ltime, lerr, f, motor) :
        """ THIS IS CRUCIAL FOR CORRECT INITIALIZATION: Initialize all the three in down-up order """
        chrono.ChFunction_Setpoint.__init__(self) 
        chrono.ChFunction_SetpointCallback.__init__(self) 
        
        # Here some specific data to be used in Get_y(),
        # add whatever you need, ex:
        self.setpoint_position_sine_amplitude = amp
        self.setpoint_position_sine_freq = freq
        self.controller_P = prop  # for our basic PID
        self.controller_D = der  # for our basic PID
        self.last_time = ltime
        self.last_error = lerr
        self.F = f
        self.linearmotor= motor  # may be useful later
    
        # Here we will compute F(t) by emulating a very basic PID scheme.
        # In practice, it works like a callback that is executed at each time step.
        # Implementation of this function is mandatory!!!
      def SetpointCallback(self, x) :
            # Trick: in this PID example, we need the following if(..)  to update PID
            # only when time changes (as the callback could be invoked more than once per timestep):
            time = x
            if (time > self.last_time) :
                dt = time - self.last_time
                # for example, the position to chase is this sine formula:
                setpo= self.setpoint_position_sine_amplitude * m.sin(self.setpoint_position_sine_freq * chrono.CH_C_2PI * x)
                error = setpo- self.linearmotor.GetMotorPos()
                error_dt = (error - self.last_error) / dt
                # for example, finally compute the force using the PID idea:
                self.F = self.controller_P * error + self.controller_D * error_dt
                self.last_time = time
                self.last_error = error
            
            return self.F
     # def SetSetpoint(self,y,x):
     #       chrono.ChFunction_Setpoint.SetSetpoint(self,y,x)
        
#      def Update(self, x):
#            y = self.SetpointCallback(x)
#            self.SetSetpoint(y, x)



# 2. Create the function from the custom class
mFcallback = MyForceClass(1.6,0.5,42000,1000,0,0,0,motor4)

# 3. Let the motor use our custom force:
motor4.SetForceFunction(mFcallback)

# EXAMPLE B.5
#
#
# - class:   ChLinkMotorLinearDriveline
# - type:    linear motor
# - control: delegated to an embedded user-defined driveline/powertrain
#
# This is the most powerful linear actuator type. It allows the creation of
# generic 1D powertrain inside this 3D motor.
# Powertrains/drivelines are defined by connecting a variable number of
# 1D objects such as ChShaft, ChClutch, ChShaftsMotor, etc. In this way, for
# example, you can represent a drive+flywheel+reducer+pulley system, hence taking into account
# of the inertia of the flywheel and the elasticity of the synchro belt without
# the complication of adding full 3D shapes that represents all parts.
# The 1D driveline is "interfaced" to the two connected threedimensional
# parts using two "inner" 1D shafts, each translating as the connected 3D part
# it is up to the user to build the driveline that connects those two shafts.
# Most often the driveline is like a graph starting at inner shaft 2 and ending
# at inner shaft 1.
# Note: in the part 2 there is an additional inner shaft that operates on rotation
# this is needed because, for example, maybe you want to model a driveline like a
# drive+screw you will anchor the drive to part 2 using this rotational shaft so
# reaction torques arising because of inner flywheel accelerations can be transmitted to this shaft.

positionB5 = chrono.ChVectorD(0, 0, 1)
guide5, slider5 = CreateSliderGuide(material, mphysicalSystem, positionB5)

# Create the motor
motor5 = chrono.ChLinkMotorLinearDriveline()

# Connect the rotor and the stator and add the motor to the system:
motor5.Initialize(slider5,               # body A (slave)
                   guide5,                # body B (master)
                   chrono.ChFrameD(positionB5) ) # motor frame, in abs. coords

mphysicalSystem.Add(motor5)

# You may want to change the inertia of 'inner' 1D shafts, ("translating" shafts: each has default 1kg)
# Note: they adds up to 3D inertia when 3D parts translate about the link shaft.
# Note: do not use too small values compared to 3D inertias: it might negatively affect
# the precision of some solvers if so, rather diminish the 3D inertia of guide/slider parts and add to these.
motor5.GetInnerShaft1lin().SetInertia(3.0)  # [kg]
motor5.GetInnerShaft2lin().SetInertia(3.0)  # [kg]
motor5.GetInnerShaft2rot().SetInertia(0.8)  # [kg/m^2]

# Now create the driveline. We want to model a drive+reducer sytem.
# This driveline must connect "inner shafts" of s1 and s2, where:
#  s1, is the 3D "slider5" part B (ex. a gantry robot gripper) and
#  s2, is the 3D "guide5"  part A (ex. a gantry robot base).
# In the following scheme, the motor is [ DRIVE ], the reducer is [ RACKPINION ],
# the shafts ( shown with symbol || to mean inertia) are:
#  S1: the 1D inner shaft for s1 robot arm (already present in ChLinkMotorLinearDriveline)
#  S2: the 1D inner shaft for s2 robot base (already present in ChLinkMotorLinearDriveline)
#  B : the shaft of the electric drive
# Note that s1 and s2 are translational degrees of freedom, differently
# from ChLinkMotorLinearDriveline.
#
#     S2rot              B                      S1lin
#    <--||---[ DRIVE ]---||-----[ RACKPINION ]----||-. 3d
# 3d <                          [            ]          s1
# s2 < S2lin                    [            ]
#    <--||----------------------[            ]
#

# Tell the motor that the inner shaft  'S2rot' is Z, orthogonal to
# the X direction of the guide (default was X, as for screw actuators)

motor5.SetInnerShaft2RotDirection(chrono.VECT_Z)  # in link coordinates

# Create 'B', a 1D shaft. This is the shaft of the electric drive, representing its inertia.

my_shaftB = chrono.ChShaft()
my_shaftB.SetInertia(0.33)  # [kg/m^2]
mphysicalSystem.Add(my_shaftB)

# Create 'DRIVE', the hispeed motor - as a simple example use a 'imposed speed' motor: this
# is the equivalent of the ChLinkMotorRotationAngle, but for 1D elements:

my_driveli = chrono.ChShaftsMotorAngle()
my_driveli.Initialize(my_shaftB,                   # B    , the rotor of the drive
                       motor5.GetInnerShaft2rot() ) # S2rot, the stator of the drive

mphysicalSystem.Add(my_driveli)

# Create a angle(time) function. It could be something as simple as
#   my_functangle = chrono.ChFunction_Ramp>(0,  180)
# but here we'll rather do a back-forth motion, made with a repetition of a sequence of 4 basic functions:

my_functsequence = chrono.ChFunction_Sequence()
my_funcsigma1 = chrono.ChFunction_Sigma(180, 0, 0.5)  # diplacement, t_start, t_end
my_funcpause1 = chrono.ChFunction_Const(0)
my_funcsigma2 = chrono.ChFunction_Sigma(-180, 0, 0.3)  # diplacement, t_start, t_end
my_funcpause2 = chrono.ChFunction_Const(0)
my_functsequence.InsertFunct(my_funcsigma1, 0.5, 1.0, True)  # fx, duration, weight, enforce C0 continuity
my_functsequence.InsertFunct(my_funcpause1, 0.2, 1.0, True)  # fx, duration, weight, enforce C0 continuity
my_functsequence.InsertFunct(my_funcsigma2, 0.3, 1.0, True)  # fx, duration, weight, enforce C0 continuity
my_functsequence.InsertFunct(my_funcpause2, 0.2, 1.0, True)  # fx, duration, weight, enforce C0 continuity
my_functangle = chrono.ChFunction_Repeat()
my_functangle.Set_fa(my_functsequence)
my_functangle.Set_window_length(0.5 + 0.2 + 0.3 + 0.2)
my_driveli.SetAngleFunction(my_functangle)

# Create the RACKPINION.
# It will connect three parts:
# - S2lin, the carrier (here the truss) to transmit reaction force,
# - B,     the in (rotational) shaft,
# - S1lin, the out (translational) shaft.

my_rackpinion = chrono.ChShaftsPlanetary()
my_rackpinion.Initialize(motor5.GetInnerShaft2lin(),  # S2lin, the carrier (truss)
                          my_shaftB,                    # B,     the input shaft
                          motor5.GetInnerShaft1lin() )  # S1lin, the output shaft

my_rackpinion.SetTransmissionRatios(-1, -1.0 / 100.0, 1)
mphysicalSystem.Add(my_rackpinion)

# Btw:  later, if you want, you can access / plot speeds and
# torques for whatever part of the driveline by putting lines like the   following
# in the  while() {...} simulation loop:
#
# GetLog() << " 1D shaft 'B' angular speed: "      << my_shaftB.GetPos_dt() << " [rad/s] \n"
# GetLog() << " 1D Drive angular speed: rot-stat " << my_driveli.GetMotorRot_dt() << " [rad/s] \n"
# GetLog() << " 1D Drive torque: "                 << my_driveli.GetMotorTorque() << " [Ns] \n"
# GetLog() << " 3D actuator speed: rot-stat " << motor5.GetMotorPos() << " [rad/s] \n"
# GetLog() << " 3D actuator force: "                 << motor5.GetMotorForce() << " [Ns] \n"
# etc.

# EXAMPLE B.6
#
# - class:   ChLinkMotorLinearPosition
# - type:    linear motor
# - control: impose a position by continuously changing it during the while{} simulation
#
# We use again the  ChLinkMotorLinearPosition as in EXAMPLE B.1, but
# this time we change its position using a "brute force" approach, that is:
# we put a line in the while{...} simulation loop that continuously changes the
# position by setting a value from some computation.
#  Well: here one might be tempted to do  motor6.SetMotionFunction(myconst) where
# myconst is a ChFunction_Const object where you would continuously change the value
# of the constant by doing  myconst.Set_yconst() in the while{...} loop this would
# work somehow but it would miss the derivative of the function, something that is used
# in the guts of ChLinkMotorLinearPosition. To overcome this, we'll use a  ChFunction_Setpoint
# function, that is able to guess the derivative of the changing setpoby doing numerical
# differentiation each time you call  myfunction.SetSetpoint().
#  Note: A more elegant solution would be to inherit our custom motion function
# from  ChFunction_SetpointCallback  as explained in EXAMPLE B.4, and then setting
# motor6.SetMotionFunction(mycallback) this would avoid polluting the while{...} loop
# but sometimes is faster to do the quick & dirty approach of this example.

positionB6 = chrono.ChVectorD(0, 0, 2)
guide6, slider6 = CreateSliderGuide(material, mphysicalSystem, positionB6)

# Create the linear motor
motor6 = chrono.ChLinkMotorLinearPosition()

# Connect the guide and the slider and add the motor to the system:
motor6.Initialize(slider6,               # body A (slave)
                   guide6,                # body B (master)
                   chrono.ChFrameD(positionB6) ) # motor frame, in abs. coords

mphysicalSystem.Add(motor6)

# Create a ChFunction to be used for the ChLinkMotorLinearPosition
# Note! look later in the while{...} simulation loop, we'll continuously
# update its value using  motor6setpoint.SetSetpoint()
motor6setpoint= chrono.ChFunction_Setpoint()
# Let the motor use this motion function:
motor6.SetMotionFunction(motor6setpoint)

#
# THE VISUALIZATION SYSTEM
#

# Create the Irrlicht visualization (open the Irrlicht device,
# bind a simple user interface, etc. etc.)
application = chronoirr.ChIrrApp(mphysicalSystem, "Motors", chronoirr.dimension2du(800, 600))

# Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
application.AddTypicalLogo()
application.AddTypicalSky()
application.AddTypicalLights()
application.AddTypicalCamera(chronoirr.vector3df(1, 3, -7))
application.AddLightWithShadow(chronoirr.vector3df(20.0, 35.0, -25.0), chronoirr.vector3df(0, 0, 0), 55, 20, 55, 35, 512,
                               chronoirr.SColorf(0.6, 0.8, 1.0))

# Use this function for adding a ChIrrNodeAsset to all items
# Otherwise use application.AssetBind(myitem) on a per-item basis.
application.AssetBindAll()

# Use this function for 'converting' assets into Irrlicht meshes
application.AssetUpdateAll()

# This is to enable shadow maps (shadow casting with soft shadows) in Irrlicht
# for all objects (or use application.AddShadow(..) for enable shadow on a per-item basis)
application.AddShadowAll()

#
# THE SOFT-REAL-TIME CYCLE
#

# Modify some setting of the physical system for the simulation, if you want
solver = chrono.ChSolverPSOR()
solver.SetMaxIterations(50)
mphysicalSystem.SetSolver(solver)

application.SetTimestep(0.005)
application.SetTryRealtime(True)

while application.GetDevice().run() :
    application.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))

    application.DrawAll()

    # Example B.6 requires the setpoto be changed in the simulation loop:
    # for example use a clamped sinusoid, just for fun:
    t = mphysicalSystem.GetChTime()
    Sp = chrono.ChMin(chrono.ChMax(2.6 * m.sin(t * 1.8), -1.4), 1.4)
    motor6setpoint.SetSetpoint(Sp, t)

    application.DoStep()

    application.EndScene()


