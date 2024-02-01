Chrono::Solidworks {#manual_chrono_solidworks}
==========================

![](http://projectchrono.org/assets/manual/carousel_chronosolidworks.jpg)

Chrono::SolidWorks is an add-in for [SolidWorks](http://www.solidworks.com) that allows to export SolidWorks models directly into Chrono.

The tool allows to:
- export entire *Assemblies*, nested *SubAssemblies* and *Parts*, together with their material and appearance properties;
- export *Mates* of the *Standard* type plus *Mehcanical>Hinge*;
- specify *collision shapes*, both through primitives or by automatic generating a contact mesh shape from the objects;
- add [Chrono motors](#ref motors), together with their control functions;

The SolidWorks model can be exported to:
- **Python**: can be run by PyChrono (also directly from SolidWorks) or parsed through the [Chrono::Parsers](@ref manual_parsers) module (still, in this case, PyChrono needs to be installed);
- **C++**: the generated files needs to be compiled together with the user project; it might be limiting but very useful for testing;
- **JSON**: this allows the deserialization of the model through the Chrono serialization feature;

The fastest way to consume the output of the SolidWorks add-in is by using the [dedicated template project](https://github.com/projectchrono/chrono-solidworks/tree/master/to_put_in_app_dir/ChronoSolidworksImportTemplate) that can be found in the add-in repository. It is also a good source of information on how to load the exported models in the various cases.


![chrono_solidworks_overview](http://projectchrono.org/assets/manual/chrono_solidworks_overview.png)


# Usage

Once installed (see the [dedicated installation guide](@ref chrono_solidworks_installation)) a new icon with the Chrono logo should appear in the SolidWorks *Task Pane* (i.e. the vertical set of icons on the right side of the SolidWorks display). All the following actions are performed through this panel.

![chrono_solidworks_panel](http://projectchrono.org/assets/manual/chrono_solidworks_panel.png)

Some prior information should be known to successfully export the model:
+ run the export only from *Assemblies*, not from single *Parts*;
+ each *SubAssembly* is considered as a single rigid body, unless it is set as Flexible;
+ Chrono motors are specified through *Coordinate Systems* objects that must be placed **at top level** (not inside *Parts*);
+ **objects do not collide** by default;
+ collision is set **on Solid Bodies**, not *Parts*;
+ the creation of **Primitive** collision shapes will convert the material of the selected body to **Air**;


## Adding Collision Shapes

Collision detection and contacts simulation are usually among the most expensive actions to be performed in a simulation. Because of this, only those bodies that are specifically flagged as "collidable" will be taken into consideration by collision algorithms.

For the same reason is good practice, whenever possible to **wrap complex bodies into primitive shapes** by creating an additional simple body around the object. The material of this wrapping object is then automatically set to **Air**.

To enable collision on a body:
1. select a *Solid Body* (not a whole *Part*!): expand the *Part* tree, look for *Solid Bodies*, pick one of them;
2. expand the Chrono::SolidWorks add-in by clicking on it;
3. choose between different collision shape types, in this order:
   1. **Primitive Shape** (the lightest to compute): if the body resembles a primitive shape (box, sphere, cylinder, etc) this is automatically recognized and attached to the body;
      Moreover, the selected body will be set to be of air material and its appearance will be changed to semi-transparent;
   2. **Mesh**: most of the bodies fall into this category, where the shapes are not so simple as primitives; a mesh is automatically generated; however, better performance can be achieved by replacing it with a hand-tailored mesh in postprocess; less robust than primitive shapes;
      a sphere sweep (of radius *Mesh Sphere Sweep Radius*) will happen on bodies, thus "inflating" the original shape for what concerns collisions; the bigger the radius, the more robust, but the less accurate will be the collision detection;
   3. **Convex Decomposition**: to be implemented...

After flagging the body to become a collision shape, a custom label will be prefixed to the solid body name. Do not modify such prefix. Moreover, please mind that the *Part* itself will be affected. So, all instances will be enabled to collision.

Some collision settings are available at the bottom of the panel.

Please be aware that this action cannot be undone by just clicking *Undo*.



## Adding Motors

Chrono motors can be added directly to the SolidWorks model by placing a *Coordinate System* (in *Assembly* tab > *Reference Geometry*). This will act as a placeholder where a proper @ref ChLinkMotor "ChLinkMotor" will be later placed during the export phase.

To add a Chrono motor:
1. expand the Chrono::SolidWorks add-in by clicking on it;
2. click on **Motors**;
3. the panel will offer various choices: pick the appropriate motor together with its governing function (see the [relative manual](motors.html#how_to_control_motors) on how to use them in Chrono);
4. select a *Coordinate System* from the tree view and click on **Add marker**; the *Coordinate System* must be at the top-level!
5. select a *Part* from the tree view and click on **Add slave body**; do the same for the master body;
6. click on **Create motor**.

A custom object will appear as a child item of the selected *Coordinate System*: by selecting it and then clicking on the **Motors** button it is possible to edit the motor properties or also to delete it.


## Customize Settings

Some settings are self-explanatory. For all of them the tooltip will offer some basic information. Here a description of those more intricated:

- **Separate .obj for each subpart**: *Parts* or *SubAssemblies* that consist of multiple solid bodies can generate either a mesh file for each single object or a unique mesh that includes them all;
- **Export Scale**: the model can be scaled prior to export through the appropriate setting: please mind that the add-in automatically recognize the units used in SolidWorks and takes care of transforming them into SI units without any further change. This scaling options are only to provide an additional scaling on top of the typical one.
- **Run Test**: if PyChrono is available on the machine the simulation can be run directly after export: click on **Save PY for testing**, then **Export to Python** and then click on **Run Test**;



# Other links

* @subpage chrono_solidworks_installation
* [Tutorials](@ref tutorial_table_of_content_chrono_solidworks)

