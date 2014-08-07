//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////
  
  

#include "vehicle_simulator.h"



// Use the namespace of Chrono

using namespace chrono;



//
// This is the program which is executed
//

int main(int argc, char* argv[])
{
		// Here create an instance of the simulator (here, the creation code of the 
		// object 'the_simulator' opens the 3D view, creates the GUI etc. See MySimulator class.
	MySimulator* the_simulator = new MySimulator;
	
		// Run the simulation cycle!! (this might last seconds, minutes, ...)
	the_simulator->Run();

		// Ok, the simulation finished (for example the user pressed 'window close' button).
		// Deleting the simulator object, will call ~MySimulator() destruction code which also
		// closes the Irrlicht 3D view, etc.
	delete the_simulator;

	return 0;
}


