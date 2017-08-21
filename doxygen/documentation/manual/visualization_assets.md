
Assets      {#visualization_assets}
====================

__Assets__ represent optional objects that can be attached to [rigid bodies](@ref rigid_bodies) to provide access to information/data such as:
* Visualization shapes
* Custom user-defined data such as electric charge, costs, sounds, etc. 


# ChAsset    {#manual_ChAsset}

The ChAsset class is the base class for all types of assets. 
See @ref chrono::ChAsset for API details.

Example: a custom-defined asset is introduced below to illustrate a case in which a user attaches 'electrical charge' to a rigid body.

~~~{.cpp}
// First declare a class inherited from ChAsset:

class ElectricParticleProperty: public ChAsset
{ 
public:
	// Data for this type of asset 
	double charge;
	
	// Default constructor with initialization
	ElectricParticleProperty()
	{
		charge = 0;
	}
};

// Create an asset object of  ElectricParticleProperty type

ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
electric_asset->charge = 100;

// ..and attach it to a body:

mbody->AddAsset(electric_asset);
~~~

Example: selecting all the assets of a certain type.

~~~{.cpp}
for (unsigned int na= 0; na<abody->GetAssets().size(); na++)
{
	auto myasset = abody->GetAssetN(na);
	if (std::shared_ptr<ElectricParticleProperty> electricasset = std::dynamic_pointer_cast<ElectricParticleProperty>(myasset))
	{
		// Do things with the electricasset, for instance
		electricasset->charge++;				
	}
}
~~~

# Visualization shapes     {#manual_ChVisualization}

There are many ready-to-use assets that define visualization shapes. Note that multiple visualization shapes can be attached to one body. 

Chrono is rendering engine agnostic. Indeed, if the [IRRLICHT module](group__irrlicht__module.html) is used,
it is up to the Irrlicht module to convert the visualization assets 
into something that can be rendered in the OpenGL view of Irrlicht. Likewise, 
if the [POSTPROCESS module](group__postprocess__module.html) is used instead, this unit is expected to convert the visualization assets into scripts with shapes for the POVray or some other rendering tool.

Visualization assets are inherited from a base class called ChVisualization.
See @ref chrono::ChVisualization for API details.

Each shape has a translation and a rotation defined with respect to the REF reference 
of the owner [body](@ref rigid_bodies). Note that this is not the COG frame, as shown in the figure below.

![](http://www.projectchrono.org/assets/manual/pic_ChAsset.png)

Examples of visualization assets:

- @ref chrono::ChSphereShape
- @ref chrono::ChBoxShape
- @ref chrono::ChCylinderShape
- @ref chrono::ChEllipsoidShape
- @ref chrono::ChConeShape
- @ref chrono::ChCapsuleShape

A special type of visualization asset is the @ref chrono::ChAssetLevel. 
An object of this type does not represent a shape. Rather, it can contain other 
visualization assets as a group. This allows the creation 
of hierarchical models. When rotating or translating a ChAssetLevel object, 
all the shapes associated with that object are moved/rotated.

Other special assets are:

- @ref chrono::ChColorAsset
- @ref chrono::ChTexture

They affect all the assets belonging to the same level by coloring/texturing them.

Example.
~~~{.cpp}
// Assume that a body_b is already available at this point. Demonstrate how to add a box.

auto mbox = std::make_shared<ChBoxShape>();

mbox->GetBoxGeometry().Pos = ChVector<>(0,-1,0);
mbox->GetBoxGeometry().Size = ChVector<>(10,0.5,10);
body_b->AddAsset(mbox);	

//Example: add a texture

auto mtexture = std::make_shared<ChTexture>;
mtexture->SetTextureFilename(GetChronoDataFile("bluewhite.png"));
body_b->AddAsset(mtexture);

// Example: add a mesh via an .obj file

auto mobjmeshfile = std::make_shared<ChObjShapeFile>();
mobjmeshfile->SetFilename("forklift_body.obj");
body_b->AddAsset(mobjmeshfile);

// Example: add a mesh with triangle data

auto mobjmesh = std::make_shared<ChTriangleMeshShape>();
mobjmesh->GetMesh()->LoadWavefrontMesh("forklift_body.obj");
body_b->AddAsset(mobjmesh);
~~~


# Examples

See also:

- [demo_irr_assets](@ref tutorial_demo_irr_assets)






