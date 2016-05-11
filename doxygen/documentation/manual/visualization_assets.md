
Visualization assets      {#visualization_assets}
====================

This section is about assets, that is, optional objects that can be attached to [rigid bodies](@ref rigid_bodies). 
Assets can be used either for attaching 
* custom user-defined data (es. electric charge, costs, sounds, etc.) 
* or to attach visualization shapes.


# ChAsset    {#manual_ChAsset}

The ChAsset class is the base class for all types of assets. 
See @ref chrono::ChAsset for API details.

The following is an example of a custom-defined asset, 
where the user wants to attach an information 'electrical charge' 
to a rigid body:

~~~{.cpp}
// First declare a class inherited from ChAsset:

class ElectricParticleProperty : public ChAsset
{ 
public:
	// data for this type of asset 
	double charge;
	
	// default constructor with initialization
	ElectricParticleProperty()
	{
		charge = 0;
	}
};

// Now create an asset object from that class 

ChSharedPtr<ElectricParticleProperty> electric_asset(new ElectricParticleProperty); 
electric_asset->charge = 100;

// ..and attach it to a body:

mbody->AddAsset(electric_asset);
~~~


Note that if you want to iterate over assets, later, 
you might need to filter some specific class of asset, 
and this can be done this way:

~~~{.cpp}
for (unsigned int na= 0; na< abody->GetAssets().size(); na++)
{
	ChSharedPtr<ChAsset> myasset = abody->GetAssetN(na);
	if (ChSharedPtr<ElectricParticleProperty> electricasset = myasset.DynamicCastTo<ElectricParticleProperty>())
	{
		// do things with the electricasset, ex print electricasset->charge;				
	}
}
~~~

# Visualization shapes     {#manual_ChVisualization}

There are many ready-to-use assets that are used to define visualization shapes.
One can attach an unlimited number of visualization shapes to a body.

This system is interface-agnostic, in the sense that it does not 
depend on a specific rendering engine. If, say, the [IRRLICHT module](@ref module_irrlicht) is used, 
it is up to the Irrlicht module to convert these visualization assets 
in something that can be rendered in the OpenGL view of Irrlicht; 
if the [POSTPROCESSING module](@ref module_postprocessing) is used instead, such unit can convert 
the visualization assets in scripts with shapes for the POVray rendering tool, etc.

Visualization assets are inherited from a common class: ChVisualization.
See @ref chrono::ChVisualization for API details.

Each shape has a translation and a rotation defined respect to the REF reference 
of the owner [body](@ref rigid_bodies):

![](pic_ChAsset.png)

There are many types of visualization assets:

- ChSphereShape
- ChBoxShape
- ChCylinderShape
- ChEllipsoidShape
- ChConeShape
- ChCapsuleShape

A special type of visualization asset it the

- ChAssetLevel

that does not represent a shape, but it can contain other 
visualization assets, as a group. This allows the creation 
of hierarchical models. When rotating or translating a ChAssetLevel, 
all other shapes are moved/rotated, because their frame is 
defined respect to the owner level.

Other special assets are:

- ChColorAsset
- ChTexture

They affect all the assets belonging to the same level, by coloring/texturing them.

Some examples. Assuming you already created a body ```body_b```:

~~~{.cpp}
// Example: add a box

ChSharedPtr<ChBoxShape> mbox (new ChBoxShape);
mbox->GetBoxGeometry().Pos = ChVector<>(0,-1,0);
mbox->GetBoxGeometry().Size = ChVector<>(10,0.5,10);
body_b->AddAsset(mbox);	

//Example: add a texture

ChSharedPtr<ChTexture> mtexture(new ChTexture);
mtexture->SetTextureFilename(GetChronoDataFile("bluwhite.png"));
body_b->AddAsset(mtexture);

// Example add a mesh (just referencing an .obj file):

ChSharedPtr<ChObjShapeFile> mobjmeshfile(new ChObjShapeFile);
mobjmeshfile->SetFilename("forklift_body.obj");
body_b->AddAsset(mobjmeshfile);

// Example add a mesh with triangle data:

ChSharedPtr<ChTriangleMeshShape> mobjmesh(new ChTriangleMeshShape);
mobjmesh->GetMesh()->LoadWavefrontMesh("forklift_body.obj");
body_b->AddAsset(mobjmesh);
~~~


# Examples

Among the many examples, look at:

- demo_irr_assets.cpp






