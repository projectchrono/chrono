//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHOBJELLIPSOIDSHAPE_H
#define CHOBJELLIPSOIDSHAPE_H

///////////////////////////////////////////////////
//
//   ChEllipsoidShape.h
//
//   Class for defining a sphere as an asset shape 
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
#include "geometry/ChCEllipsoid.h"


namespace chrono
{

/// Class for referencing a sphere shape that can be 
/// visualized in some way.

class ChApi ChEllipsoidShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChEllipsoid gellipsoid;

public:
				//
	  			// CONSTRUCTORS
				//

	ChEllipsoidShape () {};
	ChEllipsoidShape (geometry::ChEllipsoid& mellipsoid) : gellipsoid(mellipsoid) {};

	virtual ~ChEllipsoidShape () {};

				//
	  			// FUNCTIONS
				//

			// Access the sphere geometry
	geometry::ChEllipsoid& GetEllipsoidGeometry() {return gellipsoid;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
