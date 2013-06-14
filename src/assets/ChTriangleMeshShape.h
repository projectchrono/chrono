#ifndef CHOBJSHAPEFILE_H
#define CHOBJSHAPEFILE_H

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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCTriangleMeshConnected.h"

namespace chrono
{

/// Class for referencing a Wavefront/Alias .obj
/// file containing a shape that can be visualized
/// in some way.
/// The file is not load into this object: it
/// is simply a reference to the resource on the disk.

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


	geometry::ChTriangleMeshConnected GetMesh() const {return trimesh;}
	void SetMesh(const geometry::ChTriangleMeshConnected & mesh) {trimesh = mesh;}

};




//////////////////////////////////////////////////////
//////////////////////////////////////////////////////


} // END_OF_NAMESPACE____

#endif
