#ifndef CHIRRASSETCONVERTER_H
#define CHIRRASSETCONVERTER_H

//////////////////////////////////////////////////
//
//   ChIrrAssetConverter.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Functions to populate ChIrrNode objects corresponding
//   to the ChIrNodeAsset proxies that have been added to 
//   the asset list of a ChPhysicsItem.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <irrlicht.h>
#include "ChBodySceneNodeTools.h"
#include "ChIrrMeshTools.h"
#include "ChIrrNodeAsset.h"
#include "ChDisplayTools.h"
#include "ChIrrAppInterface.h"
#include "geometry/ChCSphere.h"
#include "geometry/ChCBox.h"
#include "geometry/ChCTriangleMeshSoup.h"
#include "assets/ChCamera.h"
#include "assets/ChBoxShape.h"
#include "assets/ChCylinderShape.h"
#include "assets/ChSphereShape.h"
#include "assets/ChObjShapeFile.h"
#include "assets/ChTexture.h"
#include "assets/ChVisualization.h"
#include "assets/ChAssetLevel.h"

namespace irr
{
namespace scene
{


/// Class with static functions which allow creation
/// of Irrlicht frequent 'scene nodes' like lights,
/// camera, sky box etc. with very simple statements.

class ChIrrAssetConverter
{
public: 

		//
		// DATA
		// 

			// shared meshes
	IAnimatedMesh* sphereMesh;
	//IAnimatedMesh* cubeMesh;
	//IAnimatedMesh* cylinderMesh;
	IMesh* cubeMesh;
	IMesh* cylinderMesh;
	irr::scene::ISceneManager* scenemanager;
	irr::IrrlichtDevice* mdevice;
	irr::ChIrrAppInterface* minterface;

	chrono::ChCamera* mcamera;
	bool camera_found_in_assets;


		//
		// FUNCTIONS
		// 

	ChIrrAssetConverter(irr::ChIrrAppInterface& ainterface)
	{
		minterface   = &ainterface;
		scenemanager = ainterface.GetSceneManager();
		mdevice      = ainterface.GetDevice();
	
		sphereMesh   = createEllipticalMesh(1.0,1.0,-2,+2,0, 15, 8);
		cubeMesh     = createCubeMesh(core::vector3df(2,2,2)); // -/+ 1 unit each xyz axis  
		cylinderMesh = createCylinderMesh(1,1,32); 
		

		//if (sphereMesh)
		//	sphereMesh->grab();
		if (cubeMesh)
			cubeMesh->grab();
		if (cylinderMesh)
			cylinderMesh->grab();
	}

	~ChIrrAssetConverter()
	{
		if (sphereMesh)
			sphereMesh->drop();
		if (cubeMesh)
			cubeMesh->drop();
		if (cylinderMesh)
			cylinderMesh->drop();
	}

		/// Set the directory where the cube, cylinder, etc. primitives 
		/// are stored as .obj  files (by default, it is "../data/")
		/// Must be set _before_ creating the ChIrrApp or the ChIrrAssetConverter (it is a static method)
	static void SetDefaultObjectDir(std::string mdir)
	{
		irrlicht_default_obj_dir = mdir;
	}

		/// Returns the proxy to the ChIrrNode, by scanning all assets.
		/// Note, check for the returned pointer, using mnode.IsNull(), 
		/// just in the case a proxy has not been added.
	chrono::ChSharedPtr<chrono::ChIrrNodeAsset> GetIrrNodeAsset(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		chrono::ChSharedPtr<chrono::ChIrrNodeAsset> myirrasset; // default: IsNull() will return true
		std::vector< chrono::ChSharedPtr<chrono::ChAsset> > assetlist = mitem->GetAssets();

		for (unsigned int k = 0; k < assetlist.size(); k++)
		{
			chrono::ChSharedPtr<chrono::ChAsset> k_asset = assetlist[k];
			// asset k of object i references a proxy to an irrlicht node?
			if ( k_asset.IsType<chrono::ChIrrNodeAsset>() )
			{
				myirrasset = k_asset;
			}
		}
		return myirrasset;
	}


		/// Shortcut to add and bind a ChIrrNodeAsset to an item, if
		/// it has not been added previously.
	void Bind(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		// find a ChIrrNodeAsset if there is already one...
		chrono::ChSharedPtr<chrono::ChIrrNodeAsset> irrasset;
		irrasset = GetIrrNodeAsset(mitem);

		if (irrasset.IsNull())
		{
			// add the ChIrrNodeAsset because it was not already there
			chrono::ChSharedPtr<chrono::ChIrrNodeAsset> mirr_assetpart(new chrono::ChIrrNodeAsset);
			mirr_assetpart->Bind(mitem, *minterface);
			mitem->AddAsset(mirr_assetpart);
		}

	}


		/// Shortcut to add and bind a ChIrrNodeAsset to all items in a ChSystem.
		/// If it has been already added, the existing ChIrrNodeAsset is used.
		/// NOTE. If you want a finer control on which item has an Irrlicht proxy,
		/// and which other does not need it, just use Bind() on a per-item basis..
		/// NOTE. This conversion should be done only if needed (ex. at the beginning of an
		/// animation), i.e. not too often, for performance reasons.
	void BindAll()
	{
		chrono::ChSystem* msystem = minterface->GetSystem();

		chrono::ChSystem::IteratorBodies myiter = msystem->IterBeginBodies();
		while (myiter != msystem->IterEndBodies())
		{
			Bind(*myiter);
			++myiter;
		}
		chrono::ChSystem::IteratorOtherPhysicsItems myiterB = msystem->IterBeginOtherPhysicsItems();
		while (myiterB != msystem->IterEndOtherPhysicsItems())
		{
			Bind(*myiterB);
			++myiterB;
		}
		chrono::ChSystem::IteratorLinks myiterC = msystem->IterBeginLinks();
		while (myiterC != msystem->IterEndLinks())
		{
			Bind(*myiterC);
			++myiterC;
		}
	}

		/// This function sets up the Irrlicht nodes corresponding to 
		/// the geometric assets that are found in the ChPhysicsItem 'mitem'.
		/// For example, if one has added a ChSphereShape and a ChBoxShape to 
		/// the assets of a ChBody, and a ChIrrNodeAsset too, this Update() function
		/// will prepare a ISceneNode in Irrlicht (precisely, a ChIrrNode node) and
		/// it will fill it with a spherical triangle mesh, and a box triangle mesh.
		/// NOTE. This must be done after the ChIrrNodeAsset has been created and bound,
		/// for example via Bind().
		/// NOTE. This conversion should be done only if needed (ex. at the beginning of an
		/// animation or when a shape changes), i.e. not too often, for performance reasons.
	void Update(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		CleanIrrlicht(mitem);
		PopulateIrrlicht(mitem);
	}


		/// For all items in a ChSystem, this function sets up the Irrlicht nodes 
		/// corresponding to the geometric assets that have been added to the items.
		/// NOTE. This must be done after the ChIrrNodeAsset has been created and bound,
		/// for example via Bind().
		/// NOTE. This conversion should be done only if needed (ex. at the beginning of an
		/// animation), i.e. not too often, for performance reasons.
	void UpdateAll()
	{
		chrono::ChSystem* msystem = minterface->GetSystem();

		chrono::ChSystem::IteratorBodies myiter = msystem->IterBeginBodies();
		while (myiter != msystem->IterEndBodies())
		{
			Update(*myiter);
			++myiter;
		}
		chrono::ChSystem::IteratorOtherPhysicsItems myiterB = msystem->IterBeginOtherPhysicsItems();
		while (myiterB != msystem->IterEndOtherPhysicsItems())
		{
			Update(*myiterB);
			++myiterB;
		}
		chrono::ChSystem::IteratorLinks myiterC = msystem->IterBeginLinks();
		while (myiterC != msystem->IterEndLinks())
		{
			Update(*myiterC);
			++myiterC;
		}

	}



		/// Clean all Irrlicht stuff that has been put in the ChIrrNode
		/// in a previous Update or PopulateIrrlicht operation. 
	void CleanIrrlicht(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		chrono::ChSharedPtr<chrono::ChIrrNodeAsset> irrasset;
		irrasset = GetIrrNodeAsset(mitem);

		if (!irrasset.IsNull())
		{
			irrasset->GetIrrlichtNode()->removeAll();
		}
	}

	void PopulateIrrlicht(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		camera_found_in_assets = 0;
		mcamera = 0;
		std::vector< chrono::ChSharedPtr<chrono::ChAsset> > assetlist = mitem->GetAssets();
		chrono::ChSharedPtr<chrono::ChIrrNodeAsset> myirrasset;

		// 1- Clean the ChIrrNode
		this->CleanIrrlicht(mitem);

		// 2- Find the ChIrrNodeAsset proxy
		myirrasset = GetIrrNodeAsset(mitem);

		if (myirrasset.IsNull())
			return;


		// 3- If shapes must be 'clones', put all them inside an intermediate level
		// (that will be cloned in ChIrrNode::OnAnimate), if necessary. Otherwise put shapes
		// normally inside the ChIrrNode

		irr::scene::ISceneNode* fillnode = myirrasset->GetIrrlichtNode();

		if (!fillnode)
			return;

		if(mitem->GetAssetsFrameNclones()>0)
		{
			irr::scene::ISceneNode* clonecontainer = this->scenemanager->addEmptySceneNode(myirrasset->GetIrrlichtNode());
			fillnode = clonecontainer;
		}

		// 4- populate the ChIrrNode with conversions 
		// of the geometric assets in this ChPhysicsItem

		irr::scene::ISceneNode* mnode = myirrasset->GetIrrlichtNode();

		chrono::ChFrame<> bodyframe; // begin with no rotation/translation respect to ChPhysicsItem (ex. a body)

		this->_recursePopulateIrrlicht(assetlist, bodyframe, fillnode); 

	}

void mflipSurfacesOnX(scene::IMesh* mesh) const
{
	if (!mesh)
		return;

	const u32 bcount = mesh->getMeshBufferCount();
	for (u32 b=0; b<bcount; ++b)
	{
		IMeshBuffer* buffer = mesh->getMeshBuffer(b);
		const u32 idxcnt = buffer->getIndexCount();
		u16* idx = buffer->getIndices();
		s32 tmp;

		for (u32 i=0; i<idxcnt; i+=3)
		{
			tmp = idx[i+1];
			idx[i+1] = idx[i+2];
			idx[i+2] = tmp;
		}
		const u32 vertcnt = buffer->getVertexCount();
		for (u32 i=0; i<vertcnt; i++)
		{
			core::vector3df oldnorm = buffer->getNormal(i);
			buffer->getNormal(i).X = -oldnorm.X; //mirrors normal on X 
		}
	}
}


	void _recursePopulateIrrlicht( 
							std::vector< chrono::ChSharedPtr<chrono::ChAsset> >& assetlist, 
							chrono::ChFrame<> parentframe, 
							irr::scene::ISceneNode* mnode)
	{
		chrono::ChSharedPtr<chrono::ChTexture> mtexture; // def no texture in level
		chrono::ChSharedPtr<chrono::ChVisualization> mvisual; // def no visualiz. settings in level

		// Scan assets in object and copy them as Irrlicht meshes in the ISceneNode
		for (unsigned int k = 0; k < assetlist.size(); k++)
		{
			chrono::ChSharedPtr<chrono::ChAsset> k_asset = assetlist[k];

			if ( k_asset.IsType<chrono::ChObjShapeFile>() )
			{
				chrono::ChSharedPtr<chrono::ChObjShapeFile> myobj(k_asset);
				irr::scene::IAnimatedMesh* genericMesh = this->scenemanager->getMesh(myobj->GetFilename().c_str());
				if (genericMesh)
				{
					irr::scene::IAnimatedMeshSceneNode* mchildnode = this->scenemanager->addAnimatedMeshSceneNode(genericMesh,mnode);
					mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, false);

					mchildnode->setScale(irr::core::vector3df(-1,1,1)); // because of Irrlicht being left handed!!!
					mchildnode->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false); // because of Irrlicht being left handed!!!
					//mflipSurfacesOnX(mchildnode->getMesh()); // this wold be better than disabling back culling, but it does not work!
				}
			}
			if ( k_asset.IsType<chrono::ChSphereShape>() && sphereMesh)
			{
				chrono::ChSharedPtr<chrono::ChSphereShape> mysphere(k_asset);
				irr::scene::ISceneNode* mchildnode = this->scenemanager->addAnimatedMeshSceneNode(this->sphereMesh,mnode);
				double mradius = mysphere->GetSphereGeometry().rad;
				mchildnode->setScale(core::vector3dfCH(chrono::ChVector<>(mradius,mradius,mradius)));
				mchildnode->setPosition(core::vector3dfCH(mysphere->GetSphereGeometry().center));
				mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
			}
			if ( k_asset.IsType<chrono::ChCylinderShape>() && cylinderMesh)
			{
				chrono::ChSharedPtr<chrono::ChCylinderShape> mycylinder(k_asset);
				//irr::scene::ISceneNode* mchildnode = this->scenemanager->addAnimatedMeshSceneNode(this->cylinderMesh,mnode);
				irr::scene::ISceneNode* mchildnode = this->scenemanager->addMeshSceneNode(this->cylinderMesh,mnode);
				double rad = mycylinder->GetCylinderGeometry().rad;
				chrono::ChVector<> dir = mycylinder->GetCylinderGeometry().p2 - mycylinder->GetCylinderGeometry().p1;
				double height = dir.Length();
				dir.Normalize();
				chrono::ChVector<> mx, my, mz;
				dir.DirToDxDyDz(&my,&mz,&mx); // y is axis, in cylinder.obj frame
				chrono::ChMatrix33<> mrot;
				mrot.Set_A_axis(mx,my,mz);
				chrono::ChCoordsys<> irrcylindercoords(mycylinder->GetCylinderGeometry().p1+dir*(0.5*height), mrot.Get_A_quaternion());
				ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrcylindercoords);
				core::vector3df irrsize((irr::f32)rad, (irr::f32)(0.5*height), (irr::f32)rad);
				mchildnode->setScale(irrsize);
				mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
			}
			if ( k_asset.IsType<chrono::ChBoxShape>() && cubeMesh)
			{
				chrono::ChSharedPtr<chrono::ChBoxShape> mybox(k_asset);
				//irr::scene::ISceneNode* mchildnode = this->scenemanager->addAnimatedMeshSceneNode(this->cubeMesh,mnode);
				irr::scene::ISceneNode* mchildnode = this->scenemanager->addMeshSceneNode(this->cubeMesh,mnode);
				chrono::ChCoordsys<> irrboxcoords(mybox->GetBoxGeometry().Pos, mybox->GetBoxGeometry().Rot.Get_A_quaternion());				
				mchildnode->setScale(core::vector3dfCH(mybox->GetBoxGeometry().Size));
				ChIrrTools::alignIrrlichtNodeToChronoCsys(mchildnode, irrboxcoords);
				mchildnode->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
			}

			if ( k_asset.IsType<chrono::ChTexture>() )
			{
				mtexture = k_asset;
			}
			if ( k_asset.IsType<chrono::ChVisualization>() )
			{
				mvisual = k_asset;
			}

			if ( k_asset.IsType<chrono::ChCamera>() )
			{
				this->camera_found_in_assets = true;
				chrono::ChSharedPtr<chrono::ChCamera> mycamera(k_asset);
				scene::RTSCamera* irrcamera = new scene::RTSCamera(mdevice, mnode, scenemanager,-1, -160.0f, 3.0f, 3.0f); 

				irrcamera->setPosition(core::vector3dfCH(mycamera->GetPosition()));
				irrcamera->setTarget(core::vector3dfCH(mycamera->GetAimPoint()));
				double fov_rad = mycamera->GetAngle()*CH_C_DEG_TO_RAD;
				irrcamera->setFOV((irr::f32)fov_rad); 
				irrcamera->setNearValue(0.3f);
				irrcamera->setMinZoom(0.6f);
			}
			if ( k_asset.IsType<chrono::ChAssetLevel>() )
			{
				ChSharedPtr<chrono::ChAssetLevel> mylevel(k_asset);

				std::vector< ChSharedPtr<ChAsset> >& subassetlist = mylevel->GetAssets();
				ChFrame<> subassetframe = mylevel->GetFrame();
				irr::scene::ISceneNode* subassetnode = scenemanager->addEmptySceneNode(mnode);
	
				// recurse level...
				_recursePopulateIrrlicht(subassetlist, subassetframe, subassetnode);
			}

		} // end loop on assets
		

		// if a texture has been found, apply it to all nodes of this level
		if (!mtexture.IsNull())
		{
			video::ITexture* mtextureMap = this->mdevice->getVideoDriver()->getTexture(mtexture->GetTextureFilename().c_str());
			ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
			for (; it != mnode->getChildren().end(); ++it)
			{
				(*it)->setMaterialTexture(0,mtextureMap);
			}
		}

		// if a visualization setting (color) has been set, apply it to all nodes of this level
		if (!mvisual.IsNull())
		{
			ISceneNodeList::ConstIterator it = mnode->getChildren().begin();
			for (; it != mnode->getChildren().end(); ++it)
			{
				for (unsigned int im =0; im < (*it)->getMaterialCount(); ++im)
				{
					(*it)->getMaterial(im).ColorMaterial = irr::video::ECM_NONE;
					(*it)->getMaterial(im).DiffuseColor.set((irr::u32)(255*mvisual->GetColor().A), 
														   (irr::u32)(255*mvisual->GetColor().R),
														   (irr::u32)(255*mvisual->GetColor().G), 
														   (irr::u32)(255*mvisual->GetColor().B));
				}
			}
		}

		// Set the rotation and position of the node container
		if (!(parentframe.GetCoord() == CSYSNORM))
		{
			ChIrrTools::alignIrrlichtNodeToChronoCsys(mnode, parentframe.GetCoord());
		}
	}


};








} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif // END of ChIrrAssetConverter.h

