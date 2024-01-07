
Visualization System {#visualization_system}
====================

Chrono objects - either bodies, meshes or even abstract shapes with no underlying physics - can be rendered and visualized through different rendering engines. At the same time, Chrono is not binded to any of them, thus allowing an easy extension to other rendering systems. Various [visualization systems](@ref manual_visualization) are indeed available.

Any visualization system inherits from a common base class - namely @ref chrono::ChVisualSystem "ChVisualSystem", defined in the **core** module - that always includes a pointer to a given @ref chrono::ChSystem "ChSystem". This binding allows the visual system to be informed about simulation updates, thus being able to update any visual asset according to the new bodies positions. Visual systems may allow simultaneous rendering of multiple Chrono systems, however only @ref chrono::opengl::ChVisualSystemOpenGL "opengl::ChVisualSystemOpenGL" currently supports this feature.

Together with the @ref chrono::ChVisualSystem "ChVisualSystem", Chrono offers also a wide set of renderer-agnostic _visual assets_: each visual system takes care of converting them into renderer-specific assets. These _visual assets_ are mainly represented by the @ref chrono::ChVisualShape "ChVisualShape" classes.

## Visual Shapes and Models {#visual_model_shapes}

The simplest way to add a visual shape (@ref chrono::ChVisualShape "ChVisualShape") to a physics object could be as simple as a couple of lines of code:

~~~{.cpp}
    auto visshape = chrono_types::make_shared<ChVisualShapeBox>(20, 1, 20);
    visshape->SetColor(ChColor(0.2f, 0.3f, 1.0f));

    body->AddVisualShape(visshape, ChFrame<>(ChVector<>(0, -1, 0), QUNIT));
~~~
In which we assume that `body` is of a type inheriting from @ref chrono::ChPhysicsItem "ChPhysicsItem" e.g. @ref chrono::ChBody "ChBody".

While being quite immediate, this approach is hiding most of the internal structure of the visual assets in Chrono. In fact @ref chrono::ChVisualShape "ChVisualShape"s are just a piece of the overall picture.

<h4>ChVisualModel</h4>

An object of type @ref chrono::ChVisualModel "ChVisualModel" takes care of holding (through pointers) all the visual assets of a given object.

A @ref chrono::ChVisualModel "ChVisualModel" object can be attached either to a:
+ @ref chrono::ChPhysicsItem "ChPhysicsItem" or inherited classes (most often either \ref chrono::ChBody "ChBody" or \ref chrono::fea::ChMesh "fea::ChMesh"), in this case:
  +  visual assets contained in the @ref chrono::ChVisualModel "ChVisualModel" will automatically move together with the object;
  +  a @ref chrono::ChVisualModel "ChVisualModel" is automatically added under the hood whenever a call to @ref chrono::ChPhysicsItem::AddVisualShape "AddVisualShape()" is made.
+ directly to a @ref chrono::ChVisualSystem "ChVisualSystem":
  + asset are considered fixed to ground and they will not move during simulation;
  + a @ref chrono::ChVisualModel "ChVisualModel" must be explicitely created and added to the system by calling @ref chrono::ChVisualSystem::AddVisualModel() "ChVisualSystem::AddVisualModel()"
  
Any @ref chrono::ChVisualModel "ChVisualModel" contains, among other members:
+ a list of pairs of @ref chrono::ChVisualShape "ChVisualShape"s together with their relative @ref chrono::ChFrame "ChFrame", expressing the relative position of the shape with respect to the parent object frame;
+ a list of @ref chrono::ChVisualShapeFEA "ChVisualShapeFEA" for representation of meshes.

Although it is always possible to attach shapes explicitely to the visual model through @ref chrono::ChVisualModel::AddShape "ChVisualModel::AddShape()", most of the time the average user may attach _ChVisualShape_ in one shot to any @ref chrono::ChPhysicsItem "ChPhysicsItem" (e.g. \ref chrono::ChBody "ChBody") by directly calling @ref chrono::ChPhysicsItem::AddVisualShape "ChPhysicsItem::AddVisualShape()". Same applies for @ref chrono::ChPhysicsItem::AddVisualShapeFEA "ChPhysicsItem::AddVisualShapeFEA()". Even in these cases, the commands are implicitely adding shapes to the underlying @ref chrono::ChVisualModel "ChVisualModel".

Please mind that, when attached to \ref chrono::ChBodyAuxRef "ChBodyAuxRef" the reference frame is considered to be `REF` frame, not `COG` as shown in the picture below.

![](http://www.projectchrono.org/assets/manual/pic_ChAsset.png)

<h4>ChVisualShape and ChVisualMaterial</h4>

Visual shapes inherits either from @ref chrono::ChVisualShape "ChVisualShape" or @ref chrono::ChVisualShapeFEA "ChVisualShapeFEA" and usually have names prefixed with ```ChVisualShape____```. They usually holds also a @ref chrono::geometry::ChGeometry "geometry::ChGeometry" object to describe their shape, together with one or more @ref chrono::ChVisualMaterial "ChVisualMaterial"s, defining any appearance property of the asset.

If no @ref chrono::ChVisualMaterial "ChVisualMaterial" has been explicitely added to the @ref chrono::ChVisualShape "ChVisualShape" it will get added automatically whenever the user sets a non-default value for any property of the shape. Multiple materials are usually used in combination with meshes like @ref chrono::ChVisualShapeModelFile "ChVisualShapeModelFile" where multiple materials might be listed in the input OBJ file.

Please refer to the @ref chrono::ChVisualShape "ChVisualShape" reference page to have a complete overview of all the possible derived classes.


To conclude, a more pedantic way to achieve the very same effect of the example above could be:

~~~{.cpp}
    auto body = chrono_types::make_shared<ChBody>();

    auto vismat = chrono_types::make_shared<ChVisualMaterial>(20, 1, 20);
    vismat->SetDiffuseColor(ChColor(0.2f, 0.3f, 1.0f));

    auto visshape = chrono_types::make_shared<ChVisualShapeBox>(20, 1, 20);
    visshape->SetMaterial(0, vismat);

    auto vismodel = chrono_types::make_shared<ChVisualModel>(20, 1, 20);
    vismodel->AddShape(visshape, ChFrame<>(ChVector<>(0, -1, 0), QUNIT));

    body->AddVisualModel(vismodel);
~~~

### Tutorials

Refer to [demo_IRR_assets](@ref tutorial_demo_irr_assets) to have an overview on how to apply assets to rigid bodies.

