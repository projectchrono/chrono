# =============================================================================
# PROJECT CHRONO - http://projectchrono.org
#
# Copyright (c) 2026 projectchrono.org
# All rights reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http://projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Authors: Patrick Chen
# =============================================================================
#
# Demo: adding custom ROS data pathways to a Chrono simulation from PYTHON.
#
# Chrono::ROS is schema-driven: any installed ROS 2 message type is addressed
# by its type-name string and fields are set/read by name. Adding a topic needs
# no C++, no bridge code, and no recompiling Chrono - everything below is pure
# Python, by subclassing ChROSHandler / ChROSSubscriptionCallback.
#
# Publishes the box height (std_msgs/msg/Float64) and pose
# (geometry_msgs/msg/PoseStamped); subscribes to a force command
# (geometry_msgs/msg/Vector3) applied to the box. Try:
#
#   ros2 topic echo /demo/output/pose
#   ros2 topic pub /demo/input/force geometry_msgs/msg/Vector3 "{z: 12000.0}"
#
# =============================================================================

import pychrono as chrono
import pychrono.ros as chros


# A custom publisher handler. Mirrors the Chrono 9.0 demo's MyCustomHandler
# (publishes an incrementing Int64 on "~/my_topic" at 1 Hz); the only change is
# that you address the type by name instead of touching rclcpp.
class MyCustomHandler(chros.ChROSHandler):
    def __init__(self, topic):
        super().__init__(1)  # 1 Hz
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, bridge):
        print("Creating publisher for topic", self.topic, "...")
        self.publisher = bridge.CreatePublisher(self.topic, "std_msgs/msg/Int64")
        return True

    def Tick(self, time):
        print("Publishing", self.ticker, "...")
        msg = self.publisher.NewMessage()
        msg["data"] = self.ticker
        self.publisher.Publish(msg)
        self.ticker += 1


# New in the schema-driven design: custom SUBSCRIBERS are just as easy (the 9.0
# demo had no custom subscriber). The callback fires inside ChROSManager.Update()
# on the simulation thread, so it is safe to read here.
class ForceReceiver(chros.ChROSSubscriptionCallback):
    def __init__(self):
        super().__init__()
        self.force = chrono.ChVector3d(0, 0, 0)

    def OnMessage(self, message):
        self.force = chrono.ChVector3d(message["x"], message["y"], message["z"])
        print("Received force command:", self.force)


# A subscriber handler: wire the callback and apply the stored force each step.
class ForceCommandHandler(chros.ChROSHandler):
    def __init__(self, box):
        super().__init__(0)  # tick every step
        self.box = box
        self.receiver = ForceReceiver()
        self.subscription = None
        self.accumulator = 0

    def Initialize(self, bridge):
        self.accumulator = self.box.AddAccumulator()
        self.subscription = bridge.CreateSubscription(
            "~/input/force", "geometry_msgs/msg/Vector3", self.receiver)
        return True

    def Tick(self, time):
        self.box.EmptyAccumulator(self.accumulator)
        self.box.AccumulateForce(self.accumulator, self.receiver.force, self.box.GetPos(), False)


def main():
    sys = chrono.ChSystemNSC()
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    phys_mat = chrono.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    floor = chrono.ChBodyEasyBox(20, 20, 1, 1000, True, True, phys_mat)
    floor.SetPos(chrono.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("floor")
    sys.AddBody(floor)

    box = chrono.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(chrono.ChVector3d(0, 0, 5))
    box.SetRot(chrono.QuatFromAngleAxis(0.2, chrono.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.AddBody(box)

    # The first three are built-in handlers, registered exactly as in the
    # Chrono 9.0 demo (same call interfaces): the clock (publishes /clock every
    # step), the box state (pose/twist/accel at 25 Hz under ~/box), and tf
    # (at 100 Hz).
    ros_manager = chros.ChROSManager("demo")
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))
    ros_manager.RegisterHandler(chros.ChROSTFHandler(100))
    # ... and the two application handlers: a custom Int64 publisher and the
    # force-command subscriber (the latter is new to the schema-driven design).
    ros_manager.RegisterHandler(MyCustomHandler("~/my_topic"))
    ros_manager.RegisterHandler(ForceCommandHandler(box))
    ros_manager.Initialize()

    time = 0.0
    step_size = 2e-3
    time_end = 1000.0

    realtime_timer = chrono.ChRealtimeStepTimer()
    while time < time_end:
        time = sys.GetChTime()
        if not ros_manager.Update(time, step_size):
            print("Chrono::ROS bridge node stopped; ending simulation.")
            break
        sys.DoStepDynamics(step_size)
        realtime_timer.Spin(step_size)


if __name__ == "__main__":
    main()
