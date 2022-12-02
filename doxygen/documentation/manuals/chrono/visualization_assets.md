
Visualization system      {#visualization_system}
====================

__Visualization assets__ represent optional objects that can be attached to [rigid bodies](@ref rigid_bodies) for visualization purposes.


The Chrono visualization system allows defining visualization models, shapes, and materials. Various concrete implementations are available (e.g., based on Irrlicht or OpenGL) and a new system based on VulkanSceneGraph (VSG) ius under development. 

Chrono is rendering engine agnostic. Indeed, if the [IRRLICHT module](group__irrlicht__module.html) is used, it is up to the Irrlicht module to convert the visualization assets into something that it can render. Likewise, if the [POSTPROCESS module](group__postprocess__module.html) is used instead, this unit is expected to convert the visualization assets into scripts with shapes for the POVray rendering tool.

Once a visualization system was constructed, a Chrono system can be attached to it for run-time or postprocess rendering.

The main components of a visualization system are as follows:

- A visualization __material__ (@ref chrono::ChVisualMaterial) defines colors (diffuse, ambient, specular, and emissive), textures, and other related properties.
- A visualization __shape__ (@ref chrono::ChVisualShape) is a geometric shape (primitive, curve, surface, or triangular mesh) with one or more associated visualization materials. If a shape has no associated material, a default material is used.
- A visualization __model__ (@ref chrono::ChVisualModel) is an aggregate of (pointers to) shapes and a transform which specifies the shape position relative to the model reference frame. Visualization shapes in a model are maintained in a vector of `ShapeInstance` (which is simply a typedef for a pair containing a shared pointer to a `ChVisualShape` and a `ChFrame`). Note that, currently a visualization model instance cannot be placed inside another visualization model, but that may be added in the future.
- A visualization __model instance__ (@ref chrono::ChVisualModelInstance) is a reference to a visualization model with an associated physics item.  A physics item may have an associated visualization model instance.  

@ref chrono::ChVisualSystem defines a base class for possible concrete run-time visualization systems and imposes minimal common functionality. A ChSystem is attached to a visual system using `ChVisualSystem::AttachSystem`. The Chrono physics system will then trigger automatic updates to the visualization system as needed, depending on the particular type of analysis being conducted. The visualization system is set up in such a way that derived classes may allow simultaneous rendering of multiple Chrono systems; currently only ChVisualSystemOpenGL supports this feature.

# Visualization models {#manual_vismodel}

The mechanism for defining visualization shapes and materials for a Chrono physics item is illkustrated in the following snippet:

~~~{.cpp}
    // Create a visual material and set properties
    auto vis_mat = chrono_types::make_shared<ChVisualMaterial>();
    vis_mat->SetDiffuseColor(ChColor(0.9f, 0.4f, 0.2f));
    // set other material properties
    
    // Create a visual shape and add one or more materials
    auto vis_sphere = chrono_types::make_shared<ChSphereShape>();
    sphere->GetSphereGeometry().rad = 0.5;
    sphere->AddMaterial(vis_mat);
    
    // Create a visual model and add one or more shapes
    auto vis_model = chrono_types::make_shared<ChVisualModel>();
    vis_model->AddShape(vis_sphere);
    // add more visual shapes to the model

    // Attach an instance of a visual model to a physics item
    body->AddVisualModel(vis_model);
~~~

# Visualization shapes     {#manual_visshape}

There are many ready-to-use assets that define visualization shapes. Note that multiple visualization shapes can be attached to one visualization model. 

Visualization assets are inherited from a base class called ChVisualShape.
See @ref chrono::ChVisualShape for API details.

Each shape has a translation and a rotation defined with respect to the reference frame of its owner visual model.  The visual model is then attached to the owner [body](@ref rigid_bodies) by specifying a translation and rotation relative to the REF body frame (note that this is not the COG frame, as shown in the figure below).

![](http://www.projectchrono.org/assets/manual/pic_ChAsset.png)

Examples of visualization assets:

- @ref chrono::ChSphereShape
- @ref chrono::ChBoxShape
- @ref chrono::ChCylinderShape
- @ref chrono::ChEllipsoidShape
- @ref chrono::ChConeShape
- @ref chrono::ChCapsuleShape

See [demo_irr_assets](@ref tutorial_demo_irr_assets) for examples of defining various visual shapes associated with bodies in a Chrono system.






