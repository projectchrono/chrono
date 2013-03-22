///////////////////////////////////////////////////
//
//   Demo code about 
//
//     - loading a SilidWorks .py file saved with 
//       the Chrono::Engine add-in,
//     - showing the system in Irrlicht.
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora 
// ------------------------------------------------ 
///////////////////////////////////////////////////
 
#include "physics/ChApidll.h" 
#include "irrlicht_interface/ChIrrApp.h"
#include "unit_PYTHON/ChPython.h"

#include <irrlicht.h>

// Use the namespace of Chrono

using namespace chrono;

// Use the main namespaces of Irrlicht
using namespace irr;

using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

   
 
int main(int argc, char* argv[])
{
	// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed. 
	DLL_CreateGlobals();

	// Create a Chrono::Engine physical system
	ChSystem mphysicalSystem;


	// 
	// LOAD THE SYSTEM
	//

	// The Python engine. This is necessary in order to parse the files that 
	// have been saved using the SolidWorks add-in for Chrono::Engine.

	ChPythonEngine my_python;

	try
	{
		// This is the instruction that loads the .py (as saved from SolidWorks) and 
		// fills the system:

		my_python.ImportSolidWorksSystem("../data/solid_works/test6", mphysicalSystem);  // note, don't type the .py suffic in filename..

		GetLog()<< "SYSTEM ITEMS: \n";
		mphysicalSystem.ShowHierarchy( GetLog()); 
	}
	catch (ChException myerror)
	{
		GetLog() << myerror.what();
	}

	// From this point, your ChSystem has been populated with objects and
	// assets load from the .py files. So you can proceed and fetch 
	// single items, modify them, or add constraints between them, etc.
	// For example you can add other bodies, etc.



	// 
	// THE VISUALIZATION
	//

	// Now, suppose one is interested in showing an animation of
	// the simulated system. There are different options, for instance
	// one could use the unit_POSTPROCESSING approach for rendering in 
	// POVray, or you can open an Irrlicht 3D realtime view and show
	// it, as in the following example code:

			// Create the Irrlicht visualization (open the Irrlicht device, 
			// bind a simple user interface, etc. etc.)
	ChIrrApp application(&mphysicalSystem, L"Import a SolidWorks system",core::dimension2d<u32>(800,600),false, true);


			// Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
	application.AddTypicalLogo();
	application.AddTypicalSky();
	application.AddTypicalLights();
	application.AddTypicalCamera(core::vector3df(0,4,-6));

			// ==IMPORTANT!== Use this function for adding a ChIrrNodeAsset to all items
			// in the system. These ChIrrNodeAsset assets are 'proxies' to the Irrlicht meshes.
			// If you need a finer control on which item really needs a visualization proxy in 
			// Irrlicht, just use application.AssetBind(myitem); on a per-item basis.

	application.AssetBindAll();

			// ==IMPORTANT!== Use this function for 'converting' into Irrlicht meshes the assets
			// that you added to the bodies into 3D shapes, they can be visualized by Irrlicht!

	application.AssetUpdateAll();



	// 
	// THE SIMULATION LOOP
	//

	application.SetStepManage(true);
	application.SetTimestep(0.01);
	application.SetTryRealtime(true);

	while(application.GetDevice()->run()) 
	{
		application.BeginScene();

		application.DrawAll();
		
		application.DoStep();
			
		application.EndScene();  
	}
	
 
 
	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}
  
