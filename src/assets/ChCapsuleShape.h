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

#ifndef CHCAPSULESHAPE_H
#define CHCAPSULESHAPE_H

///////////////////////////////////////////////////
//
//   ChCapsuleShape.h
//
//   Class for defining a capsule as an asset shape
//   that can be visualized in some way.
//
//   HEADER file for CHRONO,
//   Multibody dynamics engine
//
// ------------------------------------------------
//             www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCCapsule.h"


namespace chrono
{

/// Class for referencing a capsule shape that can be 
/// visualized in some way.

class ChApi ChCapsuleShape : public ChVisualization {

protected:
				//
				// DATA
				//
	geometry::ChCapsule gcapsule;

public:
				//
				// CONSTRUCTORS
				//

	ChCapsuleShape() {}
	ChCapsuleShape(geometry::ChCapsule& mcap) : gcapsule(mcap) {}

	virtual ~ChCapsuleShape() {}

				//
				// FUNCTIONS
				//

			// Access the capsule geometry
	geometry::ChCapsule& GetCapsuleGeometry() {return gcapsule;}

};


//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
