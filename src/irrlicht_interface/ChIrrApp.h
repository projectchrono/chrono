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

private:
	irr::scene::ChIrrAssetConverter* mconverter;

}; // end of  class 









} // END_OF_NAMESPACE____

#endif // END of ChIrrAppInterface.h

