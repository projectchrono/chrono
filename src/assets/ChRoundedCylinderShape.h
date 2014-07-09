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

#ifndef CHROUNDEDCYLINDERSHAPE_H
#define CHROUNDEDCYLINDERSHAPE_H

///////////////////////////////////////////////////
//
//   ChRoundedCylinderShape.h
//
//   Class for defining a rounded cylinder as an 
//   asset shape that can be visualized in some way.
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCRoundedCylinder.h"


namespace chrono
{

/// Class for referencing a rounded cylinder shape that can be 
/// visualized in some way.

class ChApi ChRoundedCylinderShape : public ChVisualization {

protected:
				//
				// DATA
				//
	geometry::ChRoundedCylinder groundedcyl;

public:
				//
				// CONSTRUCTORS
				//

	ChRoundedCylinderShape() {}
	ChRoundedCylinderShape(geometry::ChRoundedCylinder& mcap) : groundedcyl(mcap) {}

	virtual ~ChRoundedCylinderShape() {}

				//
				// FUNCTIONS
				//

			// Access the rounded cylinder geometry
	geometry::ChRoundedCylinder& GetRoundedCylinderGeometry() {return groundedcyl;}

};


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
