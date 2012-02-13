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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "assets/ChVisualization.h"
#include "geometry/ChCSphere.h"


namespace chrono
{

/// Class for referencing a Wavefront/Alias .obj
/// file containing a shape that can be visualized
/// in some way.
/// The file is not load into this object: it
/// is simply a reference to the resource on the disk.

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
