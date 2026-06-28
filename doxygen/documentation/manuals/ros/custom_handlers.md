How to Create a Custom ROS Handler {#custom_handlers}
===================================

This manual page introduces the process of creating a custom ROS handler for use with Chrono::ROS.

ROS is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. This manual page does not go into detail about ROS, so please refer to the [ROS website](http://www.ros.org/) for more detailed information. It is assumed that the reader is familiar with the basic concepts of ROS before continuing.

## What is a Handler?

A handler is a class that moves data between ROS and Chrono. It owns the publishers and subscriptions for one feature (a sensor, a body's state, a control input) and is called at a fixed rate to fill and publish messages or to apply received commands. You add a ROS pathway by subclassing `ChROSHandler` and registering an instance with the `ChROSManager`.

## Architecture

The ROS node runs in a separate process (`chrono_ros_node`) connected to the simulation over shared memory; the simulation process itself contains no ROS symbols. Handlers run in the simulation process. A message type is identified by its type-name string (e.g. `"sensor_msgs/msg/Image"`), resolved at run time from the sourced ROS environment; the bridge serializes and deserializes it, so a handler sets and reads message fields by name.

## Writing a Publisher Handler (Chrono → ROS)

Subclass `ChROSHandler`. Create the publisher in `Initialize()` and fill and publish a message in `Tick()`. `Tick()` is called at the rate passed to the constructor, measured in simulation time (`0` means every step).

```cpp
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSBridge.h"

using namespace chrono::ros;

class MyHandler : public ChROSHandler {
  public:
    MyHandler(std::shared_ptr<ChBody> body, const std::string& topic)
        : ChROSHandler(25 /* Hz */), m_body(body), m_topic(topic) {}

    bool Initialize(ChROSBridge& bridge) override {
        m_publisher = bridge.CreatePublisher(m_topic, "std_msgs/msg/Float64");
        return true;
    }

    void Tick(double time) override {
        auto msg = m_publisher->NewMessage();
        msg.SetDouble("data", m_body->GetPos().z());
        m_publisher->Publish(msg);
    }

  private:
    std::shared_ptr<ChBody> m_body;
    std::string m_topic;
    std::shared_ptr<ChROSPublisher> m_publisher;
};
```

Fields are set with a typed setter per primitive kind — `SetBool`, `SetInt`, `SetUInt`, `SetDouble`, `SetString`, `SetStringArray` — addressed by a dotted path (`"header.frame_id"`). Unset fields serialize as zero/empty. Use `SetTime(path, seconds)` for `builtin_interfaces/Time` fields. Sequences of nested messages are built with `AppendMessage(path)` (see `ChROSLidarHandler` packing `PointField`).

## Writing a Subscriber Handler (ROS → Chrono)

Create the subscription in `Initialize()` with a callback. The callback runs inside `ChROSManager::Update()` on the simulation thread — the same thread as `Tick()` — so reading or writing Chrono objects from it requires no locking.

```cpp
bool Initialize(ChROSBridge& bridge) override {
    m_subscription = bridge.CreateSubscription(
        m_topic, "geometry_msgs/msg/Twist",
        [this](const ChROSMessageView& msg) {
            m_throttle = msg.GetDouble("linear.x");
        });
    return true;
}
```

`ChROSMessageView` is the read-side counterpart: `GetDouble`/`GetInt`/`GetString`/... by path, plus `GetCount(path)` and `GetMessage(path, i)` for sequences.

## Writing Handlers in Python

Subclass `chros.ChROSHandler` and implement the same `Initialize`/`Tick` methods. Message fields use item syntax: `msg["field"] = value`.

```python
import pychrono.ros as chros

class MyHandler(chros.ChROSHandler):
    def __init__(self, body, topic):
        super().__init__(25.0)             # Hz
        self.body = body
        self.topic = topic

    def Initialize(self, bridge):
        self.pub = bridge.CreatePublisher(self.topic, "std_msgs/msg/Float64")
        return True

    def Tick(self, time):
        msg = self.pub.NewMessage()
        msg["data"] = self.body.GetPos().z
        self.pub.Publish(msg)
```

For subscribers, subclass `chros.ChROSSubscriptionCallback`, implement `OnMessage(view)`, and pass an instance to `bridge.CreateSubscription(topic, type, callback)`. Read fields with `view["field"]`.

## Bulk Data (Images, Point Clouds)

Primitive array fields (camera pixels, lidar points, any `T[]`) use the blob API, which copies the buffer once.

- **C++:** `msg.SetBlob(path, ptr, count)` (no copy; the source buffer must stay alive until `Publish()` returns within the same `Tick()`) or `msg.SetBlobCopy(path, ptr, count)`. Read with `view.GetBlob(path)`.
- **Python:** `msg["data"] = arr` (or `msg.SetArray(path, arr)`) accepts any C-contiguous buffer (numpy array, `bytes`, `bytearray`, `array.array`). Read with `view.GetBytes(path)` (a `bytes` copy) or `view.GetMemoryView(path)` (zero-copy, valid only inside the callback; wrap with `numpy.frombuffer`).

A handler can skip extraction while no one is subscribed by checking `publisher->GetSubscriptionCount()`.

## Custom Message Packages

Any message type whose package is built and sourced in the environment is usable by name, including custom packages — there is no registration step. If the package is not found, `Initialize()` fails with an error naming it. See `ChROSDriverInputsHandler`, which uses `chrono_ros_interfaces/msg/DriverInputs`.

## Quality of Service

`CreatePublisher` and `CreateSubscription` take an optional `ChROSQoS`. Use a preset — `ChROSQoS::Default()`, `ChROSQoS::SensorData()`, `ChROSQoS::Clock()`, `ChROSQoS::Latched()` (transient-local, for latched topics such as `/robot_description`) — or set reliability, durability, and history depth explicitly.

## Usage

```cpp
auto manager = chrono_types::make_shared<ChROSManager>();
manager->RegisterHandler(chrono_types::make_shared<MyHandler>(body, "~/output/height"));
manager->Initialize();

while (sys.GetChTime() < t_end) {
    sys.DoStepDynamics(step);
    if (!manager->Update(sys.GetChTime(), step))
        break;
}
```

`ChROSManager::Update()` ticks each handler at its rate and drains incoming subscriptions; it returns `false` if the node has stopped.

## Notes and Best Practices

*   **Rate.** The constructor rate is in simulation time, so a handler ticks consistently regardless of how fast the simulation runs relative to real time. Use `0` for "every step". Wall-clock pacing is the application's responsibility (`ChRealtimeStepTimer`).
*   **Time stamps.** Stamp `header.stamp` fields with the simulation time and register a `ChROSClockHandler` so downstream nodes can run with `use_sim_time:=true`.
*   **Threading.** Handler callbacks run on the simulation thread inside `Update()`. Do not spawn threads that touch Chrono objects or call into a handler.
*   **Discovery.** `bridge.DescribeType("sensor_msgs/msg/Image")` returns the field names and types; a misspelled field fails fast with the valid list.
*   **Examples.** `ChROSCameraHandler` (bulk image data), `ChROSLidarHandler` (sequences of nested messages), `ChROSDriverInputsHandler` (subscriber, custom message package), and the `demo_ROS_custom_handler` demo (C++ and Python).
