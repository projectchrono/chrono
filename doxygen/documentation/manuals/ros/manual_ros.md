Chrono::ROS Reference Manual {#manual_ros}
=================================

* [Install and build Chrono::ROS](@ref module_ros_installation)
* [Chrono::ROS overview](@ref module_ros_overview)
* [Chrono::ROS tutorials](@ref tutorial_table_of_content_chrono_ros)

More coming soon...

## Changelog as of Chrono 10.0

The Chrono::ROS bridge is now schema-driven. A message type is identified by its ROS type-name string (e.g. `sensor_msgs/msg/Image`) and serialized to and from CDR by Chrono itself over the shared-memory transport; the ROS subprocess is generic and is no longer compiled per message type. Message fields are set and read by name.

### How does this change impact me?

If your project only uses the built-in handlers (Clock, Body, TF, sensor, vehicle, robot), their constructor and method interfaces are unchanged and your project continues to work as before.

If you implemented a custom handler against the previous design, it must be rewritten. A handler is now a single `ChROSHandler` subclass that creates publishers and subscriptions on a `ChROSBridge` and sets message fields by name — there is no IPC struct, message-type enum, subprocess file, or Chrono rebuild involved. Custom handlers can now also be written in Python. See the [Custom Handlers](@ref custom_handlers) guide.

## Changelog as of Chrono 9.0

The Chrono::ROS module was refactored to resolve and prevent symbol collisions between the ROS 2 and Chrono process spaces, moving the ROS 2 node into a separate process connected to the simulation over shared memory. This separation is retained by the schema-driven bridge above.