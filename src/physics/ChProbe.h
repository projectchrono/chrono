#ifndef CHPROBE_H
#define CHPROBE_H

//////////////////////////////////////////////////
//  
//   ChProbe.h
//
//   Class for probes
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "physics/ChObject.h"


namespace chrono 
{

///
/// Class for probe objects, used to record data during
/// simulations.  
///
/// The ChProbe::Record() method will be automatically called
/// once per time step, during simulation.
///

class ChProbe : public ChObj {

public:
	ChProbe();
	~ChProbe();
	
	void Copy(ChProbe* source);

	//
	// FUNCTIONS
	//
			// Record the value.
			// Note that X and Y variables must be already set, using the
			// probe data. 
			// Usually mtime is used for X var (default), while a
			// script is used to specify the Y  variable.
			// The Record() is called for each integration step, by the 
			// system object which contains all probes.
	void Record(double mtime);	

			// if some data is recorded, delete.
	void Reset();
};
		

} // END_OF_NAMESPACE____

#endif  // END of header
