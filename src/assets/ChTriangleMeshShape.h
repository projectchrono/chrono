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
/// It also defines flags such as 'draw as wireframe', 'do backface culling' etc.
/// but remember that depending on the type of visualization system 
/// (POVray, Irrlich,etc.) these flags might not be supported.

class ChApi ChTriangleMeshShape : public ChVisualization {

protected:
				//
	  			// DATA
				//
	geometry::ChTriangleMeshConnected trimesh;

	bool wireframe;
	bool backface_cull;

	std::string name;

public:
				//
	  			// CONSTRUCTORS
				//

	ChTriangleMeshShape ()  
			{ 
				wireframe = false; 
				backface_cull = false;
			};

	virtual ~ChTriangleMeshShape () {};

				//
	  			// FUNCTIONS
				//


	geometry::ChTriangleMeshConnected& GetMesh()  {return trimesh;}
	void SetMesh(const geometry::ChTriangleMeshConnected & mesh) {trimesh = mesh;}

	bool IsWireframe() {return wireframe;}
	void SetWireframe(bool mw) {wireframe = mw;}

	bool IsBackfaceCull() {return backface_cull;}
	void SetBackfaceCull(bool mbc) {backface_cull = mbc;}

	const std::string& GetName() const     {return name;}
	void SetName(const std::string& mname) {name = mname;}
};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
