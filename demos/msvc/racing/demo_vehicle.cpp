///////////////////////////////////////////////////
//
//   Demo code about   
//
//     - modeling a complex mechanism (a quarter car model)
//     - using the ChLinkSpring to make spring-damper system
//     - using the ChLinkDistance class to reperesent 
//       long and thin massless rods, whose mass is negligible 
//       for dynamical analysis (as often happens in mechanisms)
//       so they can be modeled as 'distance' constraints
//       instead of making a thin body with small mass and two
//       spherical joints at the end (wihch would be much less
//       efficient from the computational point of view).
//
//	 CHRONO    
//   ------
//   Multibody dinamics engine
//   
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
  
  

#include "physics/ChApidll.h" 
#include "vehicle_simulator.h"



// Use the namespace of Chrono

using namespace chrono;



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{
		// In CHRONO engine, The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
		// global functions are needed.
	ChGlobals* GLOBAL_Vars = DLL_CreateGlobals();


		// Here create an instance of the simulator (here, the creation code of the 
		// object 'the_simulator' opens the 3D view, creates the GUI etc. See MySimulator class.
	MySimulator* the_simulator = new MySimulator;
	
		// Run the simulation cycle!! (this might last seconds, minutes, ...)
	the_simulator->Run();

		// Ok, the simulation finished (for example the user pressed 'window close' button).
		// Deleting the simulator object, will call ~MySimulator() destruction code which also
		// closes the Irrlicht 3D view, etc.
	delete the_simulator;

		// Remember this at the end of the program, if you started
		// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


