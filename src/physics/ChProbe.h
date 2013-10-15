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

class ChApi ChProbe : public ChObj {

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
