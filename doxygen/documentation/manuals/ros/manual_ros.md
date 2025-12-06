Chrono::ROS Reference Manual {#manual_ros}
=================================

* [Install and build Chrono::ROS](@ref module_ros_installation)
* [Chrono::ROS overview](@ref module_ros_overview)
* [Chrono::ROS tutorials](@ref tutorial_table_of_content_chrono_ros)

More coming soon...

## Changelog as of Chrono 9.0

The Chrono::ROS module has been refactored to resolve and prevent any future symbol collisions between the ROS 2 and Chrono process spaces. The new version moves the ROS 2 node into a separate process and serializes data from Chrono to ROS via shared memory (SHM). This enhances the separation between Chrono and ROS, further avoiding ROS-induced simulation slowdowns.


### How does this change impact me?

If you did not use any custom handlers in your project by implementing and overriding the `custom_handler` class, this change does not impact your project. Any project that only uses Sensor/Vehicle/Body handlers built into Chrono::ROS will continue to function without any API changes.

If you did implement a custom handler, you will have to port your handler to follow the new design. Please follow the step-by-step guide here: [Chrono::ROS page on Custom Handlers](@ref custom_handlers).