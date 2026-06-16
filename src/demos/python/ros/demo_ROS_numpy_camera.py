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
# Demo: publishing bulk array data (a camera image) from PYTHON with zero
# per-element overhead, using the buffer-protocol blob path.
#
# A primitive array field (here sensor_msgs/msg/Image.data, a uint8[]) is set
# directly from a numpy array:  msg["data"] = frame  - the whole buffer is
# copied in one shot, never element-by-element. On the receive side the data is
# read back without copying via msg.GetMemoryView(...), wrapped in
# numpy.frombuffer. This is the same mechanism the built-in camera/lidar
# handlers (Phase 5) use; nothing here is camera-specific.
#
# Watch it live with:
#   ros2 topic hz /camera/image
#   rviz2   (add an Image display on /camera/image)
#
# For the performance proof, bump WIDTH/HEIGHT to 3840x2160 (4K) below.
#
# =============================================================================

import time

import numpy as np

import pychrono.ros as chros

WIDTH = 3840
HEIGHT = 2160
RATE_HZ = 30.0


def make_frame(tick):
    """A cheap animated RGB gradient, as a contiguous (H, W, 3) uint8 array."""
    # Do the wrap-around math in integer space, then cast once to uint8 (numpy
    # 2.x rejects `% 256` directly on a uint8 array).
    xs = ((np.arange(WIDTH) + tick) % 256).astype(np.uint8)   # R sweeps horizontally, animated
    ys = (np.arange(HEIGHT) % 256).astype(np.uint8)           # G sweeps vertically
    frame = np.empty((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:, :, 0] = xs[None, :]
    frame[:, :, 1] = ys[:, None]
    frame[:, :, 2] = 128                                       # B constant
    return frame


# Publisher: fill a sensor_msgs/msg/Image from a numpy array each tick.
class CameraPublisher(chros.ChROSHandler):
    def __init__(self, topic):
        super().__init__(RATE_HZ)
        self.topic = topic
        self.publisher = None
        self.tick = 0

    def Initialize(self, bridge):
        self.publisher = bridge.CreatePublisher(self.topic, "sensor_msgs/msg/Image")
        return True

    def Tick(self, t):
        frame = make_frame(self.tick)
        msg = self.publisher.NewMessage()
        msg["header.frame_id"] = "camera"
        msg.SetTime("header.stamp", t)
        msg["height"] = HEIGHT
        msg["width"] = WIDTH
        msg["encoding"] = "rgb8"
        msg["step"] = WIDTH * 3
        msg["data"] = frame          # <-- buffer-protocol blob: one bulk copy
        self.publisher.Publish(msg)
        self.tick += 1


# Subscriber: read the image back with no copy and verify it round-trips.
class CameraReceiver(chros.ChROSSubscriptionCallback):
    def __init__(self):
        super().__init__()
        self.count = 0
        self.t0 = time.time()

    def OnMessage(self, message):
        h = int(message["height"])
        w = int(message["width"])
        # Zero-copy view of the pixel buffer; valid only inside this callback.
        view = message.GetMemoryView("data")
        frame = np.frombuffer(view, dtype=np.uint8).reshape(h, w, 3)
        self.count += 1
        if self.count % int(RATE_HZ) == 0:
            fps = self.count / (time.time() - self.t0)
            print(f"received {self.count} frames ({w}x{h} rgb8), "
                  f"~{fps:.1f} fps, last pixel[0,0]={tuple(int(c) for c in frame[0, 0])}")


class CameraEchoHandler(chros.ChROSHandler):
    """Wires the receiver to the same topic so the demo self-validates."""
    def __init__(self, topic):
        super().__init__(0)  # check every step
        self.topic = topic
        self.receiver = CameraReceiver()
        self.subscription = None

    def Initialize(self, bridge):
        self.subscription = bridge.CreateSubscription(
            self.topic, "sensor_msgs/msg/Image", self.receiver)
        return True

    def Tick(self, t):
        pass


def main():
    topic = "/camera/image"

    ros_manager = chros.ChROSManager("camera")
    ros_manager.RegisterHandler(CameraPublisher(topic))
    ros_manager.RegisterHandler(CameraEchoHandler(topic))
    ros_manager.Initialize()

    step = 1.0 / RATE_HZ
    t = 0.0
    while t < 1000.0:
        if not ros_manager.Update(t, step):
            print("Chrono::ROS bridge node stopped; ending demo.")
            break
        time.sleep(step)
        t += step


if __name__ == "__main__":
    main()
