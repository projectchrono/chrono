#ifndef CHIRRAPP_H
#define CHIRRAPP_H

//////////////////////////////////////////////////
//
//   ChIrrApp.h
//
//   FOR IRRLICHT USERS ONLY!
//
//   Class to create easily an Irrlich+Chrono::Engine 
//   application. Inherits from ChIrrAppInterface,
//   and adds features for converting C::E assets into
//   Irrlicht shapes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChIrrAppInterface.h"
#include "ChIrrAssetConverter.h"


namespace irr
{


/// Class to add some GUI to Irrlicht+Chrono::Engine
/// applications. 
/// Such basic GUI can be used to monitor solver 
/// timings, to change physical system settings easily, 
/// and so on.

class ChIrrApp : public ChIrrAppInterface
{
public: 

			/// Create the application with Irrlicht context (3D view, device, etc.)
		ChIrrApp(chrono::ChSystem* psystem, 
						const wchar_t* title =0, 
						core::dimension2d<u32> dimens = core::dimension2d<u32>(640,480),
						bool do_fullscreen = false,
						bool do_shadows = false,
						video::E_DRIVER_TYPE mydriver = video::EDT_DIRECT3D9) 
		: ChIrrAppInterface(psystem,title,dimens,do_fullscreen,do_shadows,mydriver)
		{
			mconverter = new irr::scene::ChIrrAssetConverter(*this);
		}



			/// This safely delete every Irrlicht item (including the 
			/// Irrlicht scene nodes)
	~ChIrrApp()
		{
			if (mconverter)
				delete mconverter;
		}

			/// Gets the asset converter
	irr::scene::ChIrrAssetConverter* GetAssetConverter() {return mconverter;}


		/// Shortcut to add and bind a ChIrrNodeAsset to an item, if
		/// it has not been added previously.
	void AssetBind(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		GetAssetConverter()->Bind(mitem);
	}

		/// Shortcut to add and bind a ChIrrNodeAsset to all items in a ChSystem.
		/// If it has been already added, the existing ChIrrNodeAsset is used.
		/// NOTE. If you want a finer control on which item has an Irrlicht proxy,
		/// and which other does not need it, just use Bind(myitem) on a per-item basis..
		/// NOTE. This conversion should be done only if needed (ex. at the beginning of an
		/// animation), i.e. not too often, for performance reasons.
	void AssetBindAll()
	{
		GetAssetConverter()->BindAll();
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
	void AssetUpdate(chrono::ChSharedPtr<chrono::ChPhysicsItem> mitem)
	{
		GetAssetConverter()->Update(mitem);
	}


		/// For all items in a ChSystem, this function sets up the Irrlicht nodes 
		/// corresponding to the geometric assets that have been added to the items.
		/// NOTE. This must be done after the ChIrrNodeAsset has been created and bound,
		/// for example via Bind().
		/// NOTE. This conversion should be done only if needed (ex. at the beginning of an
		/// animation), i.e. not too often, for performance reasons.
	void AssetUpdateAll()
	{
		GetAssetConverter()->UpdateAll();
	}

private:
	irr::scene::ChIrrAssetConverter* mconverter;

}; // end of  class 









} // END_OF_NAMESPACE____

#endif // END of ChIrrAppInterface.h

