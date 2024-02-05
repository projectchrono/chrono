How to Create a Custom ROS Handler {#custom_handlers}
===================================

This manual page aims to introduce the reader to the process of creating a custom ROS handler for use with Chrono::ROS. 

ROS is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. This manual page does not go into detail about ROS, so please refer to the [ROS website](http://www.ros.org/) for more detailed information. It is assumed that the reader is familiar with the basic concepts of ROS before continuing.

## What is a Handler?

A handler is a class that is responsible for converting messages between ROS and Chrono. It is the interface between the ROS world and the Chrono world. A handler is essentially a higher level abstraction that encapsulates all ROS entities, such as publishers, subscribers, and services, and provides a simple interface to interact with them. Any logic which interacts with ROS should be encapsulated in a handler. All handlers inherit from the [ChROSHandler](@ref chrono::ros::ChROSHandler) class.

## Creating a Custom Handler

To create a custom handler, you must create a new class that inherits from [ChROSHandler](@ref chrono::ros::ChROSHandler). This class must implement the following methods:

- [Initialize](@ref chrono::ros::ChROSHandler::Initialize): This method is called once when the handler is created. It is used to initialize the handler and set up any ROS entities that are needed. This is where you would create publishers, subscribers, and services, not in the constructor.
- [Tick](@ref chrono::ros::ChROSHandler::Tick): This method is called once per simulation step. It is used to update the handler and process any incoming messages.

Furthermore, the [ChROSHandler](@ref chrono::ros::ChROSHandler) takes a single argument in its constructor, which is the rate at which the [Tick](@ref chrono::ros::ChROSHandler::Tick) method is called. This is the rate at which the handler will process messages and update its state, relative to the simulation update rate.

### C++ Example

Here is an example of a simple custom handler that publishes a message to a ROS topic:

```cpp
class MyCustomHandler : public ChROSHandler {
  public:
    MyCustomHandler(const std::string& topic) : ChROSHandler(30), m_topic(topic), m_ticker(0) {}

    virtual bool Initialize(std::shared_ptr<ChROSInterface> interface) override {
        m_publisher = interface->GetNode()->create_publisher<std_msgs::msg::Int64>(m_topic, 1);
        return true;
    }

    virtual void Tick(double time) override {
        std_msgs::msg::Int64 msg;
        msg.data = m_ticker;
        m_publisher->publish(msg);
        m_ticker++;
    }

  private:
    const std::string m_topic;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr m_publisher;
    int m_ticker;
};
```

You then need to register the handler with the [ChROSManager](@ref chrono::ros::ChROSManager):

```cpp
// Create the ROS manager
auto ros_manager = chrono_types::make_shared<ChROSManager>();

// Create the custom handler
auto custom_handler = chrono_types::make_shared<MyCustomHandler>("/my_topic");

// Register the custom handler with the ROS manager
ros_manager->RegisterHandler(custom_handler);
```

### Python Example

Here is an example of a simple custom handler that subscribes to a ROS topic:

```python
class MyCustomHandler(chros.ChROSHandler):
    """This custom handler will just publish int messages to a topic."""

    def __init__(self, topic):
        super().__init__(30)  # 30 Hz

        self.topic = topic
        self.publisher: rclpy.publisher.Publisher = None

        self.ticker = 0

    def Initialize(self, interface: chros.ChROSPythonInterface) -> bool:
        self.publisher = interface.GetNode().create_publisher(Int64, self.topic, 1)

        return True

    def Tick(self, time: float):
        msg = Int64()
        msg.data = self.ticker
        self.publisher.publish(msg)
```

You then need to register the handler with the [ChROSManager](@ref chrono::ros::ChROSManager); however, because python wrapping of ROS 2 is not supported, you will instead need to use `rclpy`, which is exposed to the user through the `ChROSPythonManager`:

```python
# Create the ROS manager
ros_manager = chros.ChROSPythonManager()

# Create the custom handler
custom_handler = MyCustomHandler("/my_topic")

# Register the custom handler with the ROS manager
ros_manager.RegisterPythonHandler(custom_handler)
```

If you want to add one of the built-in C++ handlers to the manager, you can use the [RegisterHandler](@ref chrono::ros::ChROSManager::RegisterHandler) method as usual:

```python
# Add the built-in clock handler
ros_manager.RegisterHandler(chros.ChROSClockHandler())
```
