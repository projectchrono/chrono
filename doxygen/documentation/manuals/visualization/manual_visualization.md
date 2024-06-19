Chrono Visualization {#manual_visualization}
=================================

Chrono objects - either bodies, meshes, particles or even abstract shapes with no underlying physics - can be rendered and visualized through different rendering engines. At the same time, Chrono is not binded to any of them, thus allowing an easy extension to other rendering systems.

Depending on the complexity of the system and rendering requirements, we differentiate between two kind of _visualization systems_:
+ **Run-Time**: for fast and real-time rendering of the scene, often including user interfaces and additional information about the ongoing simulation:
  * @subpage irrlicht_visualization "Irrlicht Module"
  * @subpage vsg_visualization "VSG Module"
  * @subpage opengl_visualization "OpenGL Module"
+ **Offline/Postprocess**: for higher-quality renderings not fitting run-time restrictions:
  * @subpage blender_visualization "Blender Module"
  * @subpage povray_visualization "POVRay Module"

**Additional Documentation**

* [Tutorials](@ref tutorial_root)

