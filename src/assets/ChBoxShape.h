//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHOBJSBOXSHAPE_H
#define CHOBJSBOXSHAPE_H

///////////////////////////////////////////////////
//
//   ChBoxShape.h
//
//   Class for defining a box as an asset shape 
//   that can be visualized in some way.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCBox.h"


namespace chrono
{

/// Class for a box shape that can be visualized
/// in some way.

class ChApi ChBoxShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChBox gbox;	

public:
				//
	  			// CONSTRUCTORS
				//

	ChBoxShape () {};
	ChBoxShape (geometry::ChBox& mbox) : gbox(mbox) {};

	virtual ~ChBoxShape () {};

				//
	  			// FUNCTIONS
				//

			// Access the sphere geometry
	geometry::ChBox& GetBoxGeometry() {return gbox;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
