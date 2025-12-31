Overview of the YAML parser for Chrono models and simulations {#YAML_parser_overview}
===============================================================


## Reference frames {#YAML_parser_frames}

A Chrono model is specified with respect to an implied model reference frame (M). You can think of this reference frame as a global frame for the purpose of defining the model.  In other words, all body positions (location and orientation, <sub>M</sub>X<sub>B</sub>), joint frames (if joints are specified through an absolute joint frame), TSDA end point locations (if specified through absolute locations), etc. are assumed to be given relative to this model frame.

During instantiation of a YAML-specified Chrono model, the caller has the option of specifying the frame transform from the global reference frame (G) to the model frame of that instance, <sub>G</sub>X<sub>M</sub>.  This allows placing multiuple instances of the same YAML-specified Chrono model in the global scene, at different global locations and with different orientations.

The YAML parser assumes that rigid bodies are specified in the most general (and flexible) manner, namely with an arbitrary body reference frame (B). The body centroidal frame (C) is then specified through a transform (location and orientation, <sub>B</sub>X<sub>C</sub>) relative to the body reference frame. The body inertia tensor (composed of inertia moments and, optionally, inertia products) is assumed to be specified relative to the body centroidal frame. Similarly, any geometry (collision and/or visualization) attached to a rigid body is assumed to be specified relative to the body reference frame. For example, a cylindrical shape is fully specified through its radius and length, as well as the location of its center (l<sub>cyl</sub>) in the body reference frame and the direction of the cylinder axis (a<sub>cyl</sub>) expressed in the body reference frame.

<img src="http://www.projectchrono.org/assets/manual/YAML_frames.png" width="600" />
