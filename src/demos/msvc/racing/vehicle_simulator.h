#ifndef VEHICLE_SIMULATOR_H
#define VEHICLE_SIMULATOR_H

///////////////////////////////////////////////////
//
// Vehicle full simulator
//
///////////////////////////////////////////////////
 
 
#include "physics/ChApidll.h" 
#include "physics/ChSystem.h"
#include "physics/ChLinkDistance.h"
#include "irrlicht_interface/ChBodySceneNode.h"
#include "irrlicht_interface/ChBodySceneNodeTools.h" 
#include "irrlicht_interface/ChDisplayTools.h" 
#include "irrlicht_interface/ChIrrWizard.h"

#include "vehicle_car.h"
#include "vehicle_gui.h"

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



/// Class for the entire simulator.
/// An instance of this class will be created in the main() function
/// to create the 3d view, the GUI, the car, the ground, etc. - everything!

class MySimulator 
{
public:		
			//
			// DATA
			//
	IrrlichtDevice*		device;
	ChSystem*			my_system;
	ChBodySceneNode*	my_ground;
	MySimpleCar*		mycar;
	int camera_view_type;
	scene::ICameraSceneNode* camera;
	MyEventReceiver*	GUIinterface;


			/// Creation of a 'MySimulator' object. Here a 3D Irrlicht view is
			/// opened, the car object (containing Chrono::Engine bodies and constraints) is
			/// instanced, etc.
	MySimulator();

			/// Delete the simulator.
			/// Note that here all instanced contained objects must be deallocated, in
			/// reverse order respect to the allocation in the MySimulator() creation code!
	~MySimulator();
			
			/// The simulation cycle. Perform endlessly a cycle (until the user clicks on 
			/// window close) where time step integration is repeated and 3D view is redrawn.
	void Run();
			

			/// Updates the postion of the video camera in 3D scene. 
	void UpdateVideoCamera();

};


#endif // end of include guard - to avoid double .h inclusion