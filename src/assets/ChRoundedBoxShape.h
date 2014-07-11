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

#ifndef CHROUNDEDBOXSHAPE_H
#define CHROUNDEDBOXSHAPE_H

///////////////////////////////////////////////////
//
//   ChRoundedBoxShape.h
//
//   Class for defining a rounded box as an asset
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
#include "geometry/ChCRoundedBox.h"


namespace chrono
{

/// Class for referencing a rounded box shape that can be 
/// visualized in some way.

class ChApi ChRoundedBoxShape : public ChVisualization {

protected:
				//
				// DATA
				//
	geometry::ChRoundedBox groundedbox;

public:
				//
				// CONSTRUCTORS
				//

	ChRoundedBoxShape() {}
	ChRoundedBoxShape(geometry::ChRoundedBox& mcap) : groundedbox(mcap) {}

	virtual ~ChRoundedBoxShape() {}

				//
				// FUNCTIONS
				//

			// Access the rounded box geometry
	geometry::ChRoundedBox& GetRoundedBoxGeometry() {return groundedbox;}

};


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
