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

#ifndef CHTRIANGLEMESHSHAPE_H
#define CHTRIANGLEMESHSHAPE_H

///////////////////////////////////////////////////
//
//   ChTriangleMeshShape.h
//
//   Class for visualing a triangle mesh
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCTriangleMeshConnected.h"

namespace chrono
{

/// Class for referencing a triangle mesh shape that can be 
/// visualized in some way. Being a child class of ChAsset, it can
/// be 'attached' to physics items.

class ChApi ChTriangleMeshShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChTriangleMeshConnected trimesh;

public:
				//
	  			// CONSTRUCTORS
				//

	ChTriangleMeshShape ()  {};

	virtual ~ChTriangleMeshShape () {};

				//
	  			// FUNCTIONS
				//


	geometry::ChTriangleMeshConnected& GetMesh()  {return trimesh;}
	void SetMesh(const geometry::ChTriangleMeshConnected & mesh) {trimesh = mesh;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
