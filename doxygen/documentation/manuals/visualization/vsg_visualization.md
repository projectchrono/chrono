VSG Visualization (Run-Time) {#vsg_visualization}
==================================

Based on [VulkanSceneGraph (VSG)](https://vsg-dev.github.io/vsg-dev.io/), this run-time engine is currently under active development and promises to enrich the Chrono visualization with more advanced and modern visualization options.

Refer to @ref chrono::vsg3d::ChVisualSystemVSG "vsg3d::ChVisualSystemVSG" class for further info.

A typical usage of \ref chrono::vsg3d::ChVisualSystemVSG "vsg3d::ChVisualSystemVSG", that includes a lighting, camera, and background, is:

~~~{.cpp}
    auto vis = chrono_types::make_shared<ChVisualSystemVSG>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(ChVector2i(1200, 800));
    vis->SetWindowPosition(ChVector2i(100, 300));
    vis->SetWindowTitle("VSG Demo");
    vis->SetUseSkyBox(true);
    vis->AddCamera(ChVector3d(-8, 8, -16));
    vis->SetCameraAngleDeg(40);
    vis->SetLightIntensity(1.0f);
    vis->SetLightDirection(1.5 * CH_PI_2, CH_PI_4);
    vis_vsg->SetShadows(true);
~~~

[Chrono::VSG Reference](group__vsg__module.html)

[Chrono::VSG Install Guide](module_vsg_installation.html) 

<img src="http://www.projectchrono.org/assets/manual/vsg_visualization.png" class="img-responsive">