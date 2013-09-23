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

#ifndef CHOBJSPHERESHAPE_H
#define CHOBJSPHERESHAPE_H

///////////////////////////////////////////////////
//
//   ChSphereShape.h
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
#include "geometry/ChCSphere.h"


namespace chrono
{

/// Class for referencing a sphere shape that can be 
/// visualized in some way.

class ChApi ChSphereShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChSphere gsphere;	

public:
				//
	  			// CONSTRUCTORS
				//

	ChSphereShape () {};
	ChSphereShape (geometry::ChSphere& msphere) : gsphere(msphere) {};

	virtual ~ChSphereShape () {};

				//
	  			// FUNCTIONS
				//

			// Access the sphere geometry
	geometry::ChSphere& GetSphereGeometry() {return gsphere;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
