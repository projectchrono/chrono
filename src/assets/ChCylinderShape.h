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

#ifndef CHCYLINDERSHAPE_H
#define CHCYLINDERSHAPE_H

///////////////////////////////////////////////////
//
//   ChCylinderShape.h
//
//   Class for defining a cylinder as an asset shape 
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
#include "geometry/ChCCylinder.h"


namespace chrono
{

/// Class for referencing a cylinder shape that can be 
/// visualized in some way.

class ChApi ChCylinderShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChCylinder gcylinder;	

public:
				//
	  			// CONSTRUCTORS
				//

	ChCylinderShape () {};
	ChCylinderShape (geometry::ChCylinder& mcyl) : gcylinder(mcyl) {};

	virtual ~ChCylinderShape () {};

				//
	  			// FUNCTIONS
				//

			// Access the sphere geometry
	geometry::ChCylinder& GetCylinderGeometry() {return gcylinder;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
