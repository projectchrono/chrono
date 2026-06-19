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
# Authors: Aaron Young, Patrick Chen
# =============================================================================
#
# Demo: how to create a custom Chrono::ROS handler in Python.
#
# Subclass chros.ChROSHandler (a SWIG director) and address any installed ROS 2
# message type by its type-name string - no rclpy, no bridge code. This is the
# same handler as the Chrono 9.0 demo (an incrementing Int64 at 1 Hz), but it now
# runs through the schema bridge instead of in-process rclcpp.
#
#   ros2 topic echo /chrono_ros_node/my_topic
#
# =============================================================================

import pychrono as ch
import pychrono.ros as chros


class MyCustomHandler(chros.ChROSHandler):
    """A custom handler that publishes incrementing Int64 messages."""

    def __init__(self, topic):
        super().__init__(1)  # 1 Hz
        self.topic = topic
        self.publisher = None
        self.ticker = 0

    def Initialize(self, bridge):
        print(f"Creating publisher for topic {self.topic} ...")
        self.publisher = bridge.CreatePublisher(self.topic, "std_msgs/msg/Int64")
        return True

    def Tick(self, time):
        print(f"Publishing {self.ticker} ...")
        msg = self.publisher.NewMessage()
        msg["data"] = self.ticker
        self.publisher.Publish(msg)
        self.ticker += 1


def main():
    # Create the system.
    sys = ch.ChSystemNSC()
    sys.SetGravitationalAcceleration(ch.ChVector3d(0, 0, -9.81))

    phys_mat = ch.ChContactMaterialNSC()
    phys_mat.SetFriction(0.5)

    floor = ch.ChBodyEasyBox(10, 10, 1, 1000, True, True, phys_mat)
    floor.SetPos(ch.ChVector3d(0, 0, -1))
    floor.SetFixed(True)
    floor.SetName("base_link")
    sys.Add(floor)

    box = ch.ChBodyEasyBox(1, 1, 1, 1000, True, True, phys_mat)
    box.SetPos(ch.ChVector3d(0, 0, 5))
    box.SetRot(ch.QuatFromAngleAxis(0.2, ch.ChVector3d(1, 0, 0)))
    box.SetName("box")
    sys.Add(box)

    # Create the ROS manager and register the handlers.
    ros_manager = chros.ChROSManager()
    ros_manager.RegisterHandler(chros.ChROSClockHandler())
    ros_manager.RegisterHandler(chros.ChROSBodyHandler(25, box, "~/box"))

    tf_handler = chros.ChROSTFHandler(30)
    tf_handler.AddTransform(floor, floor.GetName(), box, box.GetName())
    ros_manager.RegisterHandler(tf_handler)

    ros_manager.RegisterHandler(MyCustomHandler("~/my_topic"))

    ros_manager.Initialize()

    # Simulation loop.
    time = 0
    time_step = 1e-3
    time_end = 30

    realtime_timer = ch.ChRealtimeStepTimer()
    while time < time_end:
        sys.DoStepDynamics(time_step)
        time = sys.GetChTime()

        if not ros_manager.Update(time, time_step):
            break

        realtime_timer.Spin(time_step)


if __name__ == "__main__":
    main()
