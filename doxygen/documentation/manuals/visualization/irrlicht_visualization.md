Irrlicht Visualization (Run-Time) {#irrlicht_visualization}
==================================

The [Irrlicht](https://irrlicht.sourceforge.io/) rendering engine has been the main Chrono rendering system for real-time applications since a long time. Because of this, the \ref chrono::irrlicht::ChVisualSystemIrrlicht "irrlicht::ChVisualSystemIrrlicht" wrapper for the Irrlicht engine is currently offering the widest set of features among all the rendering engines, including real-time information about the underlying problem size, simulation options and timing, various flags to enable the rendering of link frames and forces, contact reactions and much more.

Also the MODAL module relies on Irrlicht to render mode shapes.

\ref chrono::irrlicht::ChVisualSystemIrrlicht "irrlicht::ChVisualSystemIrrlicht" is in charge of creating the visualization window, including the info panel (button `i`).

A typical usage of \ref chrono::irrlicht::ChVisualSystemIrrlicht "irrlicht::ChVisualSystemIrrlicht", that includes a minimal lighting, camera and background, is:

~~~{.cpp}
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("MyDemo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 8, 6));
    vis->AddTypicalLights();
~~~

Refer to @ref chrono::irrlicht::ChVisualSystemIrrlicht "irrlicht::ChVisualSystemIrrlicht" documentation for further info.

[Chrono::Irrlicht Reference](group__irrlicht__module.html)

[Chrono::Irrlicht Install Guide](module_irrlicht_installation.html) 

<img src="http://www.projectchrono.org/assets/manual/irrlicht_visualization.png" class="img-responsive">

