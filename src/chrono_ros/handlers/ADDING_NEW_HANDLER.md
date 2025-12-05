# Chrono::ROS Handler Implementation Guide

This guide explains how to implement new handlers for the Chrono::ROS interface.

## Architecture Overview

Chrono::ROS uses a **two-process architecture** to ensure separation and avoid symbol conflicts (particularly between VSG visualization and ROS 2 libraries as of Chrono 9.0).

1.  **Main Process (Chrono Simulation)**: Runs the physics simulation. It extracts data from Chrono objects and serializes it into raw bytes. It contains **NO** ROS symbols.
2.  **Subprocess (ROS Node)**: Runs the ROS node. It receives raw bytes via IPC (Inter-Process Communication), deserializes them, and publishes standard ROS messages. It contains **NO** Chrono physics symbols.
3. The Chrono::ROS Interface attempts to reserve(not necessarily use) at least 1GB of /dev/shm to support 4k Chrono:Sensor Image Transport, if you are using this in Docker, please make sure to override the default 16MB SHM size applied by docker.

### The "Handler" Concept

A "Handler" is the bridge between these two worlds. To add a new feature (e.g., publishing a new sensor type), you must implement a Handler that spans both processes.

A complete Handler consists of 4 files:
1.  `ChROS<Name>Handler.h`: The class definition (Main Process).
2.  `ChROS<Name>Handler.cpp`: The data extraction and serialization logic (Main Process).
3.  `ChROS<Name>Handler_ipc.h`: The **shared** data structure definition (Both Processes).
4.  `ChROS<Name>Handler_ros.cpp`: The ROS publishing/subscribing logic (Subprocess).

---

## The Shared IPC Header (`*_ipc.h`)

This is the most critical file. It defines the "contract" between the Chrono process and the ROS process.

*   **Location**: Typically alongside your handler files (e.g., `src/chrono_ros/handlers/sensor/`).
*   **Content**: Plain C++ structs (POD - Plain Old Data).
*   **Restrictions**:
    *   **NO** pointers.
    *   **NO** `std::string`, `std::vector`, or other dynamic containers.
    *   **NO** ROS or Chrono headers (please use primitive types like `double`, `float`, `char[]`).

**Example (`ChROSCameraHandler_ipc.h`):**
```cpp
namespace chrono {
namespace ros {
namespace ipc {

struct CameraData {
    char topic_name[128]; // Fixed size string
    char frame_id[64];
    uint32_t width;
    uint32_t height;
    uint32_t step;
    // Data follows immediately after this struct in the byte stream
};

} // namespace ipc
} // namespace ros
} // namespace chrono
```

---

## Implementation Pattern: Publisher (Chrono → ROS)

Use this pattern for sensors or state reporting (e.g., Camera, GPS, Body State).

### 1. Main Process (`.h` / `.cpp`)

Inherit from `ChROSHandler`. Your job is to implement `GetSerializedData`.

**Header (`ChROSMyHandler.h`):**
```cpp
class CH_ROS_API ChROSMyHandler : public ChROSHandler {
public:
    // ... constructor ...
    
    // 1. Initialize: Validate inputs (e.g. check topic name)
    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override;
    
    // 2. Message Type: Return the enum value for your message
    virtual ipc::MessageType GetMessageType() const override { return ipc::MessageType::MY_DATA_TYPE; }
    
    // 3. Serialize: Pack data into bytes
    virtual std::vector<uint8_t> GetSerializedData(double time) override;
};
```

**Implementation (`ChROSMyHandler.cpp`):**
```cpp
#include "ChROSMyHandler_ipc.h" // Include the shared struct
#include <cstring> // For std::memcpy

std::vector<uint8_t> ChROSMyHandler::GetSerializedData(double time) {
    // Optional: Implement throttling logic here if needed
    // if (time - m_last_time < 1.0 / m_update_rate) return {};

    // 1. Extract data from Chrono object
    auto data = m_my_object->GetData();

    // 2. Pack into IPC struct
    ipc::MyDataStruct msg;
    strncpy(msg.topic_name, m_topic_name.c_str(), sizeof(msg.topic_name) - 1);
    msg.value = data;

    // 3. Serialize to vector
    // Note: In production code, use a member variable m_buffer to avoid reallocations
    std::vector<uint8_t> buffer(sizeof(ipc::MyDataStruct));
    std::memcpy(buffer.data(), &msg, sizeof(ipc::MyDataStruct));
    return buffer;
}
```

### 2. Subprocess (`_ros.cpp`)

This file is **only** compiled into the ROS node executable. It uses `rclcpp` to publish the data.

**Implementation (`ChROSMyHandler_ros.cpp`):**
```cpp
#include "chrono_ros/ChROSHandlerRegistry.h"
#include "ChROSMyHandler_ipc.h"
#include "std_msgs/msg/float64.hpp" // ROS message header

namespace chrono {
namespace ros {

// The callback function
void PublishMyDataToROS(const uint8_t* data, size_t data_size, 
                        rclcpp::Node::SharedPtr node, 
                        ipc::IPCChannel* channel) {
    
    // 1. Deserialize
    if (data_size < sizeof(ipc::MyDataStruct)) return;
    const auto* msg_ipc = reinterpret_cast<const ipc::MyDataStruct*>(data);

    // 2. Create/Get Publisher (using static map for persistence)
    static std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers;
    std::string topic = msg_ipc->topic_name;
    
    if (publishers.find(topic) == publishers.end()) {
        publishers[topic] = node->create_publisher<std_msgs::msg::Float64>(topic, 10);
    }

    // 3. Publish ROS Message
    std_msgs::msg::Float64 msg_ros;
    msg_ros.data = msg_ipc->value;
    publishers[topic]->publish(msg_ros);
}

// 4. Register the handler
CHRONO_ROS_REGISTER_HANDLER(MY_DATA_TYPE, PublishMyDataToROS)

} // namespace ros
} // namespace chrono
```

---

## Implementation Pattern: Subscriber (ROS → Chrono)

Use this pattern for control inputs (e.g., Driver Inputs, Robot Control). This is a **Bidirectional** flow.

1.  **Setup Phase**: Main Process sends a "Setup" message (containing the topic name) to Subprocess.
2.  **Runtime Phase**: Subprocess receives ROS message → Sends IPC message to Main Process → Main Process applies to Chrono object.

### 1. Main Process (`.h` / `.cpp`)

**Header (`ChROSMySubscriber.h`):**
```cpp
class CH_ROS_API ChROSMySubscriber : public ChROSHandler {
public:
    // ...
    
    // 1. Handle Incoming: Apply data from ROS to Chrono
    virtual void HandleIncomingMessage(const ipc::Message& msg) override;
    
    // 2. Enable bidirectional support
    virtual bool SupportsIncomingMessages() const override { return true; }
    
    // 3. Serialize: Sends the SETUP message (topic name) once
    virtual std::vector<uint8_t> GetSerializedData(double time) override;
};
```

**Implementation (`ChROSMySubscriber.cpp`):**
```cpp
std::vector<uint8_t> ChROSMySubscriber::GetSerializedData(double time) {
    // Send topic name ONLY ONCE to trigger subscriber creation in subprocess
    if (!m_setup_sent) {
        m_setup_sent = true;
        return std::vector<uint8_t>(m_topic_name.begin(), m_topic_name.end());
    }
    return {}; // No data sent after setup
}

void ChROSMySubscriber::HandleIncomingMessage(const ipc::Message& msg) {
    // Unpack data sent back from subprocess
    const auto* data = msg.GetPayload<ipc::MyControlStruct>();
    
    // Apply to Chrono physics
    m_my_object->SetControl(data->control_value);
}
```

### 2. Subprocess (`_ros.cpp`)

**Implementation (`ChROSMySubscriber_ros.cpp`):**
```cpp
static rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr g_sub;
static ipc::IPCChannel* g_ipc_channel = nullptr; // Needed to send data back

// ROS Callback
void OnMessageReceived(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!g_ipc_channel) return;

    // 1. Pack IPC struct
    ipc::MyControlStruct data;
    data.control_value = msg->data;

    // 2. Send back to Main Process
    ipc::Message ipc_msg(ipc::MessageType::MY_DATA_TYPE, 0, sizeof(data), 
                         reinterpret_cast<const uint8_t*>(&data), sizeof(data));
    g_ipc_channel->SendMessage(ipc_msg);
}

// Setup Function (called once when Main Process sends topic name)
void SetupMySubscriber(const uint8_t* data, size_t size, 
                       rclcpp::Node::SharedPtr node, 
                       ipc::IPCChannel* channel) {
    g_ipc_channel = channel; // Store channel for callback
    
    std::string topic(reinterpret_cast<const char*>(data), size);
    g_sub = node->create_subscription<std_msgs::msg::Float64>(
        topic, 10, OnMessageReceived);
}

CHRONO_ROS_REGISTER_HANDLER(MY_DATA_TYPE, SetupMySubscriber)
```

---

## Registration Steps

To integrate your new handler into the build system and IPC framework, follow these 2 steps.

### 1. Add Message Type Enum
**File**: `src/chrono_ros/ipc/ChROSIPCMessage.h`

Add a new value to `enum class MessageType`.

**Why?** This unique ID allows the Subprocess to know which callback function to execute when it receives a message.

```cpp
enum class MessageType : uint32_t {
    // ...
    MY_DATA_TYPE = 15, // Add this
    // ...
};
```

### 2. Update CMakeLists.txt
**File**: `src/chrono_ros/CMakeLists.txt`

Add your `_ros.cpp` file to the `chrono_ros_node` target sources.

**Why?** The ROS-side logic (publishing/subscribing) must be compiled into the separate `chrono_ros_node` executable, not the main Chrono library.

**CRITICAL**: If you forget this step, your handler will compile, but you will see "No handler registered for message type" warnings at runtime because the registration macro in your `_ros.cpp` file was never executed in the subprocess.

```cmake
target_sources(chrono_ros_node PRIVATE
    # ...
    handlers/sensor/ChROSMyHandler_ros.cpp
)
```

### 3. Update SWIG Interface (Optional)
**File**: `src/chrono_swig/interface/ros/ChModuleROS.i`

If you want to use your handler in Python (PyChrono), you must register it in the SWIG interface file.

**Why?** SWIG needs to know about your C++ class to generate the Python wrapper.

1.  **Include Header**: Add `#include "chrono_ros/handlers/sensor/ChROSMyHandler.h"` in the C++ block (inside `%{ ... %}`).
2.  **Enable Shared Pointer**: Add `%shared_ptr(chrono::ros::ChROSMyHandler)` in the shared pointers section.
3.  **Include SWIG Definition**: Add `%include "../../../chrono_ros/handlers/sensor/ChROSMyHandler.h"` in the include section.

> **Note**: You do **NOT** need to modify `ChROSManager.cpp`. The manager is generic and will automatically handle your new class as long as it inherits from `ChROSHandler`.

---

## Usage (In Your Application)

Finally, to use your new handler in a simulation:

```cpp
// 1. Create the handler
auto my_handler = chrono_types::make_shared<ChROSMyHandler>(...);

// 2. Register with Manager
// Why? This tells the Manager to call your handler's GetSerializedData() every update.
ros_manager->RegisterHandler(my_handler);
```

## Best Practices

*   **Throttling**: Implement rate limiting in `GetSerializedData` using `m_update_rate`. Return an empty vector if it's not time to publish. This ease the stress on ROS(its DDS middleware) since simulation can run many times faster than realtime in some cases.
*   **Memory**: Avoid reallocating vectors in the loop. Use a member variable `std::vector<uint8_t> m_buffer` and `resize()` it.
*   **Thread Safety**: In bidirectional handlers, `HandleIncomingMessage` is called from the main thread, but ensure your Chrono object modifications are safe if you are using multi-threaded stepping.
*   **Examples**:
    *   See `ChROSCameraHandler` for complex data (images).
    *   See `ChROSDriverInputsHandler` for bidirectional control.

