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

#ifndef CHROUNDEDCONESHAPE_H
#define CHROUNDEDCONESHAPE_H

///////////////////////////////////////////////////
//
//   ChRoundedConeShape.h
//
//   Class for defining a rounded cone as an asset
//   shape that can be visualized in some way.
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCRoundedCone.h"


namespace chrono
{

/// Class for referencing a rounded cone shape that can be 
/// visualized in some way.

class ChApi ChRoundedConeShape : public ChVisualization {

protected:
				//
				// DATA
				//
	geometry::ChRoundedCone groundedcone;

public:
				//
				// CONSTRUCTORS
				//

	ChRoundedConeShape() {}
	ChRoundedConeShape(geometry::ChRoundedCone& mcap) : groundedcone(mcap) {}

	virtual ~ChRoundedConeShape() {}

				//
				// FUNCTIONS
				//

			// Access the rounded cone geometry
	geometry::ChRoundedCone& GetRoundedConeGeometry() {return groundedcone;}

};


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
