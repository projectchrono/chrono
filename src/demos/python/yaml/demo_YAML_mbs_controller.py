# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2025 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Radu Serban
# =============================================================================
#
# Demo for using the Chrono YAML parser in python
#
# =============================================================================
#
# Simple demo for illustrating the use of an external controller to activate
# a Chrono multibody system specified through a YAML model file.
#
# The model consists of an inverted pendulum on a moving cart (which slides on
# a horizontal prismatic joint). A PID controller applies a horizontal force
# to the cart in order to maintain the pendulum vertical, while moving the cart
# to a prescribed target location.  The target location switches periodically.
#
# Note: PID controller is not perfectly tuned. This demo is for illustration
#       purposes only.
#
# =============================================================================

import pychrono as chrono
import pychrono.parsers as parsers
import pychrono.vsg3d as vsg
import errno
import os
import math
import enum

# -----------------------------------------------------------------------------

# PID controller for an inverted pendulum, implementing the ChLoadControllerinterface.
# This controller can therefore be used to set a body load controller.
class InvertedPendulumLoadController(parsers.ChLoadController):
    def __init__(self, travel_dist, switch_period):
        super().__init__()
        self.m_x_cart = travel_dist
        self.m_a_pend = 0
        self.m_switch_period = switch_period
        self.m_switch_time = switch_period
        self.m_K_cart = chrono.VNULL
        self.m_K_pend = chrono.VNULL
        self.m_e_cart = chrono.VNULL
        self.m_e_pend = chrono.VNULL
        self.m_force = chrono.VNULL

    def SetGainsCart(self, Kp, Ki, Kd):
        self.m_K_cart = chrono.ChVector3d(Kp, Ki, Kd)
    
    def SetGainsPend(self, Kp, Ki, Kd): 
        self.m_K_pend = chrono.ChVector3d(Kp, Ki, Kd)

    def GetLoad(self):
        return self.m_force

    # Initialize the controller
    def Initialize(self, parser, model_instance):
        self.m_cart = parser.FindBodyByName("cart", 0)
        self.m_pend = parser.FindBodyByName("pendulum", 0)
    
    # Synchronize controller at given time: at a switch time, flip target for cart location
    def Synchronize(self, time):
        if time >= self.m_switch_time:
            self.m_x_cart = -self.m_x_cart
            self.m_switch_time = self.m_switch_time + self.m_switch_period
            print("Switch at time = ", time, " New target = ", self.m_x_cart)
            self.m_e_cart = chrono.VNULL
            self.m_e_pend = chrono.VNULL

    # Advance controller state and calculate output cart force
    def Advance(self, step):
        # Current cart location and pendulum angle
        x_cart = self.m_cart.GetPos().x
        dir = self.m_pend.TransformDirectionLocalToParent(chrono.ChVector3d(0, 0, 1))
        a_pend = math.atan2(-dir.x, dir.z)

        # Calculate current errors and derivatives
        e_cart = x_cart - self.m_x_cart
        e_pend = a_pend - self.m_a_pend

        # Calculate current error derivatives
        self.m_e_cart.z = self.m_cart.GetPosDt().x
        self.m_e_pend.z = -self.m_pend.GetAngVelLocal().y

        # Calculate current error integrals (trapezoidal rule)
        self.m_e_cart.y += (self.m_e_cart.x + e_cart) * step / 2
        self.m_e_pend.y += (self.m_e_pend.x + e_pend) * step / 2

        # Cache new errors
        self.m_e_cart.x = e_cart
        self.m_e_pend.x = e_pend

        # Calculate PID output
        F_cart = chrono.Vdot(self.m_K_cart, self.m_e_cart);
        F_pend = chrono.Vdot(self.m_K_pend, self.m_e_pend);

        self.m_force.x = F_cart + F_pend;

# PID controller for an inverted pendulum, implementing both the ChMotorController interface.
# This controller can therefore be used to set a motor actuation.
class InvertedPendulumMotorController(parsers.ChMotorController):
    def __init__(self, travel_dist, switch_period):
        super().__init__()
        self.m_x_cart = travel_dist
        self.m_a_pend = 0
        self.m_switch_period = switch_period
        self.m_switch_time = switch_period
        self.m_K_cart = chrono.VNULL
        self.m_K_pend = chrono.VNULL
        self.m_e_cart = chrono.VNULL
        self.m_e_pend = chrono.VNULL
        self.m_force = chrono.VNULL

    def SetGainsCart(self, Kp, Ki, Kd):
        self.m_K_cart = chrono.ChVector3d(Kp, Ki, Kd)
    
    def SetGainsPend(self, Kp, Ki, Kd): 
        self.m_K_pend = chrono.ChVector3d(Kp, Ki, Kd)

    def GetActuation(self): 
        return self.m_force.x

    # Initialize the controller
    def Initialize(self, parser, model_instance):
        self.m_cart = parser.FindBodyByName("cart", 0)
        self.m_pend = parser.FindBodyByName("pendulum", 0)
    
    # Synchronize controller at given time: at a switch time, flip target for cart location
    def Synchronize(self, time):
        if time >= self.m_switch_time:
            self.m_x_cart = -self.m_x_cart
            self.m_switch_time = self.m_switch_time + self.m_switch_period
            print("Switch at time = ", time, " New target = ", self.m_x_cart)
            self.m_e_cart = chrono.VNULL
            self.m_e_pend = chrono.VNULL

    # Advance controller state and calculate output cart force
    def Advance(self, step):
        # Current cart location and pendulum angle
        x_cart = self.m_cart.GetPos().x
        dir = self.m_pend.TransformDirectionLocalToParent(chrono.ChVector3d(0, 0, 1))
        a_pend = math.atan2(-dir.x, dir.z)

        # Calculate current errors and derivatives
        e_cart = x_cart - self.m_x_cart
        e_pend = a_pend - self.m_a_pend

        # Calculate current error derivatives
        self.m_e_cart.z = self.m_cart.GetPosDt().x
        self.m_e_pend.z = -self.m_pend.GetAngVelLocal().y

        # Calculate current error integrals (trapezoidal rule)
        self.m_e_cart.y += (self.m_e_cart.x + e_cart) * step / 2
        self.m_e_pend.y += (self.m_e_pend.x + e_pend) * step / 2

        # Cache new errors
        self.m_e_cart.x = e_cart
        self.m_e_pend.x = e_pend

        # Calculate PID output
        F_cart = chrono.Vdot(self.m_K_cart, self.m_e_cart);
        F_pend = chrono.Vdot(self.m_K_pend, self.m_e_pend);

        self.m_force.x = F_cart + F_pend;

# -----------------------------------------------------------------------------

class ControllerType(enum.IntEnum):
    LOAD = 1
    MOTOR = 2

# -----------------------------------------------------------------------------

# Set output root directory
chrono.SetChronoOutputPath("../DEMO_OUTPUT/")

type = ControllerType.LOAD

if type == ControllerType.LOAD:
    model_yaml_filename = chrono.GetChronoDataFile("yaml/mbs/inverted_pendulum_load.yaml")
else:
   model_yaml_filename = chrono.GetChronoDataFile("yaml/mbs/inverted_pendulum_motor.yaml")
sim_yaml_filename = chrono.GetChronoDataFile("yaml/mbs/simulation_mbs.yaml")

# Create YAML parser object
parser = parsers.ChParserMbsYAML(model_yaml_filename, sim_yaml_filename, True)

# Create the Chrono system and populate it with the model    
sys = parser.CreateSystem()
parser.Populate(sys)

# Extract information from parsed YAML files
model_name = parser.GetName()
time_end = parser.GetEndtime()
time_step = parser.GetTimestep()
real_time = parser.EnforceRealtime()
render = parser.Render()
render_fps = parser.GetRenderFPS()
enable_shadows = parser.EnableShadows()
output = parser.Output()
output_fps = parser.GetOutputFPS()

# Create a controller corresponding to the "cart_controller" definition in the YAML model file and specify that it
# should be used for the first instance of the MBS model.
if type == ControllerType.LOAD:
    controller = InvertedPendulumLoadController(2.0, 20.0)
    controller.SetGainsCart(5, 0, -0.5)
    controller.SetGainsPend(-150, -100, -10)
    parser.AttachLoadController(controller, "cart_controller", 0)
else:
    controller = InvertedPendulumMotorController(2.0, 20.0)
    controller.SetGainsCart(5, 0, -0.5)
    controller.SetGainsPend(-150, -100, -10)
    parser.AttachMotorController(controller, "cart_controller", 0)

# Create the VSG visualization system
if render:
    vis = vsg.ChVisualSystemVSG()
    vis.AttachSystem(sys)
    vis.SetWindowSize(chrono.ChVector2i(1200, 800))
    vis.SetWindowPosition(chrono.ChVector2i(100, 300))
    vis.SetWindowTitle("YAML model - " + model_name)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.AddCamera(chrono.ChVector3d(0, -5, 0.5), chrono.ChVector3d(0, 0, 0.5))
    vis.SetCameraAngleDeg(40)
    vis.SetLightIntensity(1.0)
    vis.SetLightDirection(-chrono.CH_PI_4, chrono.CH_PI_4)
    vis.EnableShadows(enable_shadows)
    vis.ToggleAbsFrameVisibility()
    vis.SetAbsFrameScale(2.0)
    vis.Initialize()

# Create output directory
if output:
    out_dir = chrono.GetChronoOutputPath() + "YAML_MBS_CONTROLLER/"
    if not os.path.exists(out_dir):
        try:
            os.mkdir(out_dir)
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                print("Error creating output directory ")
    parser.SetOutputDir(out_dir)

# Simulation loop
time = 0
render_frame = 0
output_frame = 0

while True:
    if render:
        if not vis.Run():
            break
        if time >= render_frame / render_fps:
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            render_frame += 1

    parser.DoStepDynamics()

    time += time_step
